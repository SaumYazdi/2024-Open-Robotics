"""
Usage:
    - Camera.get_distance(): Returns the ball's estimated distance
                                from the camera in centimetres.
    - Camera.get_angle():    Returns the angle of the ball relative
                                to the camera in radians.
"""

from collections import deque
import numpy as np
import cv2
import imutils
from time import perf_counter, sleep
from typing import Callable
from classes import *
from math import atan2, degrees

try:
    from picamera2 import Picamera2
    from libcamera import controls
    DEVICE = "pi"
except ImportError:
    from imutils.video import VideoStream
    DEVICE = "pc"

RESIZE_WIDTH = 640
BUFFER_SIZE = 64
MAX_BALL_JUMP = 720 # If ball is detected, how close the new pos must be for it the current position to update
BALL_TRACK_COLOUR = (0, 0, 0)
MIN_RADIUS = 6

BALL_DIAMETER = 6.9 #cm
CALIBRATION_DISTANCE = 30 #cm

if DEVICE == "pc":
    # ORANGE_LOWER = (1, 114, 245)
    # ORANGE_UPPER = (8, 203, 255)
    LERP_STEP = 0.4
    FPS_UPDATE = 120
elif DEVICE == "pi":
    # ORANGE_LOWER = (1, 235, 230)
    # ORANGE_UPPER = (11, 255, 255)
    LERP_STEP = 0.8
    FPS_UPDATE = 24

from os.path import join, dirname
import json
save_path = join(dirname(__file__), "save.json")

with open(save_path, "r") as f:
    data = json.load(f)
    dist = data["dist"]
    angle = data["angle"]
    color = data["color"]
    f.close()
    
def get_dist(radius):
    return dist["k"] * pow(radius, dist["a"])

def get_angle(x, distance):
    normal_x = x / distance
    estimated_x = angle["m"] * normal_x + angle["c"]
    return atan2(estimated_x, distance)

av = {}
def timer(func):
    def wrapper(*args, **kwargs):
        start = perf_counter()
        data = func(*args, **kwargs)
        name = func.__name__
        dt = (perf_counter() - start) * 1000
        if name not in av:
            av[name] = [dt, 1]
        else:
            av[name][0] += dt
            av[name][1] += 1
        print(f"'{name}' {av[name][0] / av[name][1]:.2f}")
        return data
    return wrapper

"""
CAMERA CONTROLS
{'HdrMode': (0, 4, 0), 'Contrast': (0.0, 32.0, 1.0), 'Brightness': (-1.0, 1.0, 0.0),
'ColourGains': (0.0, 32.0, None), 'AeFlickerPeriod': (100, 1000000, None),
'AwbMode': (0, 7, 0), 'AeFlickerMode': (0, 1, 0), 'AeExposureMode': (0, 3, 0),
'AwbEnable': (False, True, None), 'Saturation': (0.0, 32.0, 1.0),
'StatsOutputEnable': (False, True, False), 'FrameDurationLimits': (33333, 120000, None), 
'AeEnable': (False, True, None), 'ExposureTime': (0, 66666, None), 'AeMeteringMode': (0, 3, 0),
'NoiseReductionMode': (0, 4, 0), 'AnalogueGain': (1.0, 16.0, None), 'ScalerCrop': ((0, 0, 0, 0),
(65535, 65535, 65535, 65535), (0, 0, 0, 0)), 'AeConstraintMode': (0, 3, 0),
'ExposureValue': (-8.0, 8.0, 0.0), 'Sharpness': (0.0, 16.0, 1.0)}
"""

class Camera:
    def __init__(self, window_name: str,
            preview: bool = False, draw_detections: bool = False):
                
        if DEVICE == "pi":
            self.video_stream = Picamera2()
            raw_config = self.video_stream.sensor_modes[5]
            config = self.video_stream.create_video_configuration(
                main={"format": 'XRGB8888', "size": [640//2, 480//2]},
                raw=raw_config,
                buffer_count=2,
                controls={'FrameRate': 144},
            )
            self.video_stream.configure(config)
            # self.video_stream.set_controls({'HdrMode': controls.HdrModeEnum.SingleExposure})
            
        elif DEVICE == "pc":
            self.video_stream = VideoStream(src=0).start()

        self.window_name = window_name
        self.preview = preview
        self.draw_detections = draw_detections
        self.fps = None
                
        self.pos = None
        self.radius = None
        # self.velocity = None
        # self.points = deque(maxlen=BUFFER_SIZE)
        
        self.distance = None
        self.angle = None

        self.image = None
        self.color_mask = None

        self.update_events = []
        
        self.set_color_bounds(color["lower"], color["upper"])
        
    def set_color_bounds(self, lower, upper):
        self.orange_lower = tuple(lower)
        self.orange_upper = tuple(upper)
    
    def get_mask(self, frame):
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        self.color_mask = hsv.copy()
        
        mask = cv2.inRange(hsv, self.orange_lower, self.orange_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        return mask

    def get_distance(self):
        if self.radius is None:
            return None
        if self.distance is None:
            return get_dist(self.radius)
        return self.distance

    def get_angle(self):
        if self.radius is None:
            return None
        if self.angle is None:
            return get_angle(self.pos.x, self.get_distance())
        return self.distance
    
    def compute(self, frame):
        mask = self.get_mask(frame)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        center = None
        
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            pos = Vector(x, y)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            
            if radius > MIN_RADIUS:
                if self.pos is None:
                    self.pos = pos
                if self.radius is None:
                    self.radius = radius

                if dist_squared(self.pos, pos) < MAX_BALL_JUMP ** 2:
                    # old_pos = self.pos.copy()
                    self.pos.x = lerp(self.pos.x, x, LERP_STEP)
                    self.pos.y = lerp(self.pos.y, y, LERP_STEP)
                    # self.velocity = Vector(self.pos.x - old_pos.x, self.pos.y - old_pos.y)

                    self.radius = lerp(self.radius, radius, LERP_STEP)
                    self.distance = get_dist(self.radius)
                    self.angle = degrees(get_angle(self.pos.x, self.distance))
                    
                if self.draw_detections:
                    # Draw True Circle Pos and Centroid
                    # cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # cv2.circle(frame, center, 2, BALL_TRACK_COLOUR, -1)
                    scale = self.radius * .006
                    
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    text_pos = [int(self.pos.x - 140 * scale), int(self.pos.y - 25 * scale)]
                    thickness = 1
                    cv2.putText(frame, f"dist: {self.distance:.2f}cm", text_pos, font, scale, (0, 0, 0), thickness, cv2.LINE_AA)
                    cv2.putText(frame, f"angle: {self.angle:.2f} deg", (text_pos[0], int(text_pos[1] + 50 * scale)), font, scale, (0, 0, 0), thickness, cv2.LINE_AA)

        else:
            self.pos = None
            self.radius = None
            self.distance = None
            self.angle = None

        # self.points.appendleft(self.pos.int() if self.pos else None)
        return frame
    
    def read(self):
        if DEVICE == "pc":
            return self.video_stream.read()
        return self.video_stream.capture_array()
    
    def _update(self):
        # start = perf_counter()
        frame = self.read()
        # print(f"'read frame' took {(perf_counter() - start) * 1000:.2f}ms")
        
        if frame is None:
            return
        
        # Important or detection will break!
        # Resize to detection resolution
        if type(RESIZE_WIDTH) == int:
            frame = imutils.resize(frame, width=RESIZE_WIDTH)
        frame = self.compute(frame)

        if self.draw_detections:
            # Draw Ball Trail
            # for i in range(1, len(self.points)):
                # if self.points[i - 1] is None or self.points[i] is None:
                    # continue
                
                # thickness = int(np.sqrt(BUFFER_SIZE / float(i + 1)) * 2.5)
                # cv2.line(frame, self.points[i - 1], self.points[i], BALL_TRACK_COLOUR, thickness)
            
            if self.pos and self.radius:
                cv2.circle(frame, self.pos.int(), int(self.radius), BALL_TRACK_COLOUR, 1, cv2.LINE_AA)

                # Draw Velocity Vector
                # point2 = Vector(self.pos.x + self.velocity.x * 50, self.pos.y + self.velocity.y * 50)
                # cv2.line(frame, self.pos.int(), point2.int(), (255, 0, 50), 3)

        if self.preview == True:
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord("q"):
                return
        
        return frame

    def set_update(self, event: Callable):
        self.update_events.append(event)

    def start(self, delay=2) -> None:
        if DEVICE == "pi":
            self.video_stream.start()
            
        sleep(delay)
        
        count = 10
        start = perf_counter()
        for i in range(0,count) :
            request = self.video_stream.capture_request()
            metadata = request.get_metadata()
            request.release()
        fps = float(count/(perf_counter() - start))
        print("Metadata only", " Spf:", 1./fps, " Fps:", fps)
        
        ticks = 1
        prev = perf_counter()
        while True:
            if (ticks % FPS_UPDATE) == 0:
                now = perf_counter()
                dt = now - prev
                prev = now
                self.fps = FPS_UPDATE / dt 

                if self.preview:
                    cv2.setWindowTitle(self.window_name, f"FPS: {self.fps}")

            self.image = self._update()
            if self.image is None:
                break

            for event in self.update_events:
                event()
            
            ticks += 1

        self.stop()

    def stop(self) -> None:
        if DEVICE == "pi":
            self.video_stream.close()
        elif DEVICE == "pc":
            self.video_stream.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    camera = Camera("Ball Detector")
    camera.start()
