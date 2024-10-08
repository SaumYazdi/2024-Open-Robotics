"""
Usage:
    - Camera.get_distance(): Returns the ball's estimated distance
                                from the camera in centimetres.
    - Camera.get_angle():    Returns the angle of the ball relative
                                to the camera in radians.
"""

import cv2
import imutils
from time import perf_counter, sleep
from typing import Callable
from classes import *
from math import atan2, degrees, sqrt, sin, cos, pi
import numpy
import colorsys
from threading import Thread
import socket

if socket.gethostname() == "0F00SG0224200C":
    from imutils.video import VideoStream
    DEVICE = "pc"
else:
    from picamera2 import Picamera2
    DEVICE = "pi"

print(f"Running camera on {('PC', 'Raspberry Pi')[int(DEVICE == 'pi')]}")
    

RESIZE_WIDTH = 640
BUFFER_SIZE = 64
MAX_BALL_JUMP = 720 # If ball is detected, how close the new pos must be for it the current position to update
BALL_TRACK_COLOUR = (0, 0, 0)
MIN_RADIUS = 6

BALL_DIAMETER = 6.9 # cm
CALIBRATION_DISTANCE = 30 # cm

if DEVICE == "pc":
    LERP_STEP = 0.4
    FPS_UPDATE = 120
elif DEVICE == "pi":
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

DOWN_FACING_CAMERA = 0xfa
FRONT_FACING_CAMERA = 0xee
     
# Centre mask to cover wiring but expose the dribbler section
r = 130
w = 640
h = 480
innerMaskRadius = 117
outerMaskRadius = 169
center = (int(camera_center[0]), int(camera_center[1]))
circlePoints = [center]
lowerAngle = 20 # bottom angle
upperAngle = 340 # top angle
for i in range(lowerAngle, upperAngle, 5):
    a = i * pi / 180
    circlePoints.append((int(center[0] + outerMaskRadius * cos(a)), int(center[1] + outerMaskRadius * sin(a))))
innerMask = [
    center,
    (int(center[0] + innerMaskRadius * cos(lowerAngle * pi / 180)), int(center[1] + innerMaskRadius * sin(lowerAngle * pi / 180))),
    (int(center[0] + innerMaskRadius * cos((upperAngle - 5) * pi / 180)), int(center[1] + innerMaskRadius * sin((upperAngle - 5) * pi / 180)))
]

def rgb_to_hsv(rgb) -> tuple:
    normal_rgb = [i / 255 for i in rgb]
    normal_hsv = colorsys.rgb_to_hsv(*normal_rgb)
    return (round(360 * normal_hsv[0]), round(255 * normal_hsv[1]), round(255 * normal_hsv[2]))

print("On", DEVICE)

class DownFacingCamera:
    def __init__(self, window_name: str,
            preview: bool = False, draw_detections: bool = False,
            camera_port: int = 0, detect_back: bool = False):
        
        if DEVICE == "pi":
            print(f"Starting on Picamera number {camera_port}") 
            self.video_stream = Picamera2(camera_port)
            # [print(mode) for mode in CAMERA_SENSOR_MODES]
            raw_config = CAMERA_SENSOR_MODES[1]
            raw_config["fps"] = 60
            config = self.video_stream.create_video_configuration(
                main={"format": 'XRGB8888', "size": [640, 480]},
                raw=raw_config,
                buffer_count=6,
                controls={'FrameRate': 60},
            )
            self.video_stream.configure(config)
            
        elif DEVICE == "pc":
            self.video_stream = VideoStream(src=0).start()

        self.detect_back = detect_back
        
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
        self.mask_radius = None
        with open(save_path, "r") as f:
            data = json.load(f)
            self.mask_radius = data["maskRadius"]

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
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        lower = (216, 0, 0)
        upper = (255, 50, 5)
        mask = cv2.inRange(rgb, lower, upper)
        # mask_color = cv2.cvtColor(mask, cv2.COLOR_BGR2RGB)
        # cv2.imshow("mask", cv2.bitwise_and(rgb, mask_color))
        # key = cv2.waitKey(1) & 0xFF

        return mask
        
        if key == ord("q"):
            self.stop()
            quit()
        
        cv2.drawContours(hsv, [numpy.array([ (0, 0), (r, 0), (0, r) ])], 0, (0, 255, 0), -1)
        cv2.drawContours(hsv, [numpy.array([ (0, h), (r, h), (0, h - r) ])], 0, (0, 255, 0), -1)
        cv2.drawContours(hsv, [numpy.array([ (w, h), (w - r, h), (w, h - r) ])], 0, (0, 255, 0), -1)
        cv2.drawContours(hsv, [numpy.array([ (w, 0), (w - r, 0), (w, r) ])], 0, (0, 255, 0), -1)
        
        cv2.drawContours(hsv, [numpy.array(circlePoints)], 0, (0, 255, 0), -1)
        cv2.drawContours(hsv, [numpy.array(innerMask)], 0, (255, 255, 0), -1)
        
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        with open(save_path, "r") as f:
            data = json.load(f)
        skin_lower = data["skin"]["lower"]
        skin_upper = data["skin"]["upper"]
        skin = cv2.inRange(hsv, numpy.array(skin_lower), numpy.array(skin_upper))
        skin = cv2.erode(skin, None, iterations=2)
        skin = cv2.dilate(skin, None, iterations=2)
        
        filtered_mask = cv2.bitwise_and(mask, cv2.bitwise_not(skin))
        
        return filtered_mask

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
                
            # if self.mask_radius:
                # frame = cv2.circle(frame,
                    # center=self.camera_center, radius=self.mask_radius,
                    # color=(0, 255, 0), thickness=1)
                
        if len(contours) > 0:
            
            # Merge contours into a convex contour to be fitted with ellipse
            points = []
            
            # Loop over contours from largest to smallest area
            sorted_contours = sorted(contours, key=lambda x: x.size, reverse=True)
            pX = pY = None
            
            for i in range(len(sorted_contours)):
                cnt = sorted_contours[i]
                
                # Skip over contour if size is too small
                if cnt.size < 25:
                    continue
                    
                # Compute center of contour
                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue
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
                
                if c.size > 4 * 2:
                    ellipse = cv2.fitEllipse(c)
                    center, size, angle = ellipse
                    
                    # Not actually radius, i just cbf changing the variable name
                    self.pos = Vector(center)
                    delta_pos = self.pos.x - self.camera_center[0], self.pos.y - self.camera_center[1]
                    self.angle = -atan2(delta_pos[1], delta_pos[0])
                    self.radial_distance = sqrt(delta_pos[0]**2 + delta_pos[1]**2)
                    self.radius = size[0] * size[1]
                    
                    x = int(self.camera_center[0] + self.mask_radius + 10)
                    y = int(self.camera_center[1])
                    red = frame[y, x][2]
                    green = frame[y, x][1]
                    blue = frame[y, x][0]
                    try:
                        if red > 80 and green < 50 and blue < 50:
                            self.distance = 12
                        else:
                            self.distance = get_dist(self.radius / self.radial_distance)
                    except PermissionError:
                        pass
                    scale = size[0] * .006
                    
                    with open(save_path, "r") as f:
                        data = json.load(f)
                        back = data["back"]
                    
                    if self.detect_back:
                        x = back[0]
                        y = back[1]
                        red = frame[y, x][2]
                        
                        if red > 240:
                            self.distance = 14
                        
                        cv2.drawMarker(frame, back, (255, 255, 0))
            
                    # Draw contours, ellipse and center point
                    if self.draw_detections:
                        
                        for i in range(len(contours)):
                            cv2.drawContours(frame, contours, i, (0, 255, 0))
                            
                        cv2.ellipse(frame, ellipse, (255, 255, 255), 1, cv2.LINE_AA)
                        cv2.drawMarker(frame, [int(center[0]), int(center[1])], (0, 255, 0))
                        
        else:
            self.pos = None
            self.radius = None
            self.distance = None
            self.angle = None

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
            # Praying that the robot does not stop during gameplay
            # try:
            if (ticks % FPS_UPDATE) == 0:
                now = perf_counter()
                dt = now - prev
                prev = now
                self.fps = FPS_UPDATE / dt 

                # if self.preview:
                    # cv2.setWindowTitle(self.window_name, f"FPS: {self.fps}")

            self.image = self._update()
            if self.image is None:
                break

            # if self.preview == True:
                # cv2.imshow(self.window_name, self.image)
                
            for event in self.update_events:
                event()

            # except Exception as exc:
                # print(exc)
            
            ticks += 1

        self.stop()

    def stop(self) -> None:
        if DEVICE == "pi":
            self.video_stream.close()
        elif DEVICE == "pc":
            self.video_stream.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    threads = []
    
    camera1 = DownFacingCamera("Camera1", preview=True, draw_detections=True, camera_port=0, detect_back=True)
    camera2 = DownFacingCamera("Camera2", preview=True, draw_detections=True, camera_port=1)
    
    sleep(1)
    
    def show_cameras():
        while True:
            if camera1.image is not None: cv2.imshow("FrontBack", camera1.image)
            if camera2.image is not None: cv2.imshow("SideToSide", camera2.image)
            cv2.waitKey(1)
    
    threads.append(Thread(target=camera1.start))
    threads.append(Thread(target=camera2.start))
    threads.append(Thread(target=show_cameras))
    
    for thread in threads:
        thread.start()
    
