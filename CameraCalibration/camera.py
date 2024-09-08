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
from math import atan2, degrees, sqrt
import numpy
import colorsys

# try:
from picamera2 import Picamera2
DEVICE = "pi"
# except ImportError:
    # from imutils.video import VideoStream
    # DEVICE = "pc"
print(f"Running camera on {('PC', 'Raspberry Pi')[int(DEVICE == 'pi')]}")
    

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
    camera_center = data["center"]
    f.close()
    
def get_dist(radius):
    global dist
    try:
        with open(save_path, "r") as f:
            data = json.load(f)
            dist = data["dist"]
    except json.JSONDecodeError:
        pass
    return dist["k"] * pow(radius, dist["a"])

def get_angle(x, distance):
    global angle
    
    try:
        with open(save_path, "r") as f:
            data = json.load(f)
            angle = data["angle"]
    except json.JSONDecodeError:
        pass
        
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

"""
220 degree camera sensor modes
`Picamera2().sensor_modes`
"""
CAMERA_SENSOR_MODES = [
    {'format': 'SGBRG10_CSI2P', 'unpacked': 'SGBRG10', 'bit_depth': 10, 'size': (640, 480), 'fps': 58.92, 'crop_limits': (16, 0, 2560, 1920), 'exposure_limits': (134, 1103219, None)},
    {'format': 'SGBRG10_CSI2P', 'unpacked': 'SGBRG10', 'bit_depth': 10, 'size': (1296, 972), 'fps': 43.25, 'crop_limits': (0, 0, 2592, 1944), 'exposure_limits': (92, 760636, None)},
    {'format': 'SGBRG10_CSI2P', 'unpacked': 'SGBRG10', 'bit_depth': 10, 'size': (1920, 1080), 'fps': 30.62, 'crop_limits': (348, 434, 1928, 1080), 'exposure_limits': (118, 969249, None)},
    {'format': 'SGBRG10_CSI2P', 'unpacked': 'SGBRG10', 'bit_depth': 10, 'size': (2592, 1944), 'fps': 15.63, 'crop_limits': (0, 0, 2592, 1944), 'exposure_limits': (130, 1064891, None)}
]

CAMERA_WIDTH, CAMERA_HEIGHT = 2592, 1944

def rgb_to_hsv(rgb) -> tuple:
    normal_rgb = [i / 255 for i in rgb]
    normal_hsv = colorsys.rgb_to_hsv(*normal_rgb)
    return (round(360 * normal_hsv[0]), round(255 * normal_hsv[1]), round(255 * normal_hsv[2]))
    
class Camera:
    def __init__(self, window_name: str,
            preview: bool = False, draw_detections: bool = False):
                
        if DEVICE == "pi":
            self.video_stream = Picamera2()
            [print(mode) for mode in CAMERA_SENSOR_MODES]
            raw_config = CAMERA_SENSOR_MODES[1]
            raw_config["fps"] = 60
            config = self.video_stream.create_video_configuration(
                main={"format": 'XRGB8888', "size": [640, 480]},
                raw=raw_config,
                buffer_count=6,
                controls={'FrameRate': 60},
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
        self.radial_distance = None
        # self.velocity = None
        # self.points = deque(maxlen=BUFFER_SIZE)
        
        self.distance = None
        self.angle = None

        self.image = None
        self.color_mask = None
        self.camera_center = camera_center

        self.update_events = []
        
        self.set_color_bounds(color["lower"], color["upper"])
        
    def set_color_bounds(self, lower, upper):
        self.orange_lower = tuple(lower)
        self.orange_upper = tuple(upper)
        self.hsv_lower = (lower[0] / 2, lower[1], lower[2])
        self.hsv_upper = (upper[0] / 2, upper[1], upper[2])
        # hsv = rgb_to_hsv(self.orange_lower[::-1])
        # self.hsv_lower = (hsv[0] * 179 / 360, hsv[1], hsv[2])
        # hsv = rgb_to_hsv(self.orange_upper[::-1])
        # self.hsv_upper = (hsv[0] * 179 / 360, hsv[1], hsv[2])
    
    def get_mask(self, frame):
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # cv2.rectangle(hsv, (0, 0, 50, 50), self.hsv_lower, -1)
        # cv2.imshow("hsv", hsv)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        return mask

    def get_distance(self):
        # if self.radius is None:
            # return None
        # if self.distance is None:
            # return get_dist(self.radius)
        return self.distance

    def get_angle(self):
        # if self.radius is None:
            # return None
        # if self.angle is None:
            # return get_angle(self.pos.x, self.get_distance())
        return self.angle
    
    def compute(self, frame):
        mask = self.get_mask(frame)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        center = None
        
        if self.draw_detections:
            cv2.drawMarker(frame, self.camera_center, (0, 255, 0))
                
        if len(contours) > 0:
            
            # Merge contours into a convex contour to be fitted with ellipse
            points = []
            
            # Loop over contours from largest to smallest area
            sorted_contours = sorted(contours, key=lambda x: x.size, reverse=True)
            pX = pY = None
            
            for i in range(len(sorted_contours)):
                cnt = sorted_contours[i]
                
                # Skip over contour if size is too small
                if cnt.size < 20:
                    continue
                    
                # Compute center of contour
                M = cv2.moments(cnt)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                if pX is None:
                    dist_to_last_contour = 0
                else:
                    dist_to_last_contour = sqrt((cX - pX)**2 + (cY - pY)**2)
                    
                # If the distance to the last (valid) contour is too far, then skip
                if dist_to_last_contour > 120:
                    continue
                pX, pY = cX, cY
                if self.draw_detections:
                    cv2.drawMarker(frame, (cX, cY), (255, 0, 0))
                for pt in cnt:
                    points.append(pt)
            
            if len(points) > 4:
                c = cv2.convexHull(numpy.array(points, dtype=numpy.int32))
                
                if c.size > 4:
                    ellipse = cv2.fitEllipse(c)
                    center, size, angle = ellipse
                    
                    # Not actually radius, i just cbf changing the variable name
                    self.pos = Vector(center)
                    delta_pos = self.pos.x - self.camera_center[0], self.pos.y - self.camera_center[1]
                    self.angle = -atan2(delta_pos[1], delta_pos[0])
                    self.radial_distance = sqrt(delta_pos[0]**2 + delta_pos[1]**2)
                    self.radius = size[0] * size[1]
                    self.distance = get_dist(self.radius / self.radial_distance)
                    scale = size[0] * .006
                    
                    if self.draw_detections:
                        
                        for i in range(len(contours)):
                            cv2.drawContours(frame, contours, i, (0, 255, 0))
                            
                        cv2.ellipse(frame, ellipse, (255, 255, 255), 1, cv2.LINE_AA)
                        cv2.drawMarker(frame, [int(center[0]), int(center[1])], (0, 255, 0))
                        
                        # font = cv2.FONT_HERSHEY_SIMPLEX
                        # text_pos = [int(self.pos.x - 140 * scale), int(self.pos.y - 25 * scale)]
                        # thickness = 1
                        # cv2.putText(frame, f"dist: {self.distance:.2f}cm", text_pos, font, scale, (230, 230, 230), thickness, cv2.LINE_AA)
                        # cv2.putText(frame, f"angle: {self.angle:.2f} deg", (text_pos[0], int(text_pos[1] + 50 * scale)), font, scale, (0, 0, 0), thickness, cv2.LINE_AA)

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
    camera = Camera("Ball Detector", True, True)
    camera.start()
