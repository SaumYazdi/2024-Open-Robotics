# "env/bin/python" CameraCalibration/main.py

from collections import deque
import numpy as np
import cv2
import imutils
from time import perf_counter
from typing import Callable
from classes import *
try:
    from picamera2 import Picamera2
    DEVICE = "pi"
except ImportError:
    from imutils.video import VideoStream
    DEVICE = "pc"

BUFFER_SIZE = 64
FPS_UPDATE = 24
MAX_BALL_JUMP = 720 # If ball is detected, how close the new pos must be for it the current position to update
BALL_TRACK_COLOUR = (0, 150, 255)
BALL_DIAMETER = 6.9 # centimetres
CALIBRATION_DISTANCE = 30

class Camera:
    if DEVICE == "pc":
        orange_lower = (1, 114, 240)
        orange_upper = (8, 203, 255)
    elif DEVICE == "pi":
        orange_lower = (1, 235, 230)
        orange_upper = (11, 255, 255)

    def __init__(self, window_name: str, preview: bool = True):
        self.points = deque(maxlen=BUFFER_SIZE)
        
        if DEVICE == "pi":
            self.video_stream = Picamera2()
            config = self.video_stream.create_still_configuration(
                main={"format": 'XRGB8888', "size": [640, 320]}
            )
            self.video_stream.configure(config)
        elif DEVICE == "pc":
            self.video_stream = VideoStream(src=0).start()

        self.window_name = window_name
        self.preview = preview

        self.pos = None
        self.radius = None
        self.velocity = None

        self.update_events = []
            
    def get_mask(self, frame):
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Construct a mask for the color orange, then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, self.orange_lower, self.orange_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        return mask

    def compute(self, frame):
        mask = self.get_mask(frame)
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        center = None
        
        if len(contours) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            pos = Vector(x, y)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            
            if radius > 10:
                if self.preview:
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(frame, center, 2, BALL_TRACK_COLOUR, -1)

                if self.pos is None:
                    self.pos = pos
                if self.radius is None:
                    self.radius = radius

                if dist_squared(self.pos, pos) < MAX_BALL_JUMP ** 2:
                    old_pos = self.pos.copy()
                    self.pos.x = lerp(self.pos.x, x)
                    self.pos.y = lerp(self.pos.y, y)
                    self.velocity = Vector(self.pos.x - old_pos.x, self.pos.y - old_pos.y)

                    self.radius = lerp(self.radius, radius)
                    
        else:
            self.pos = None
            self.radius = None

        self.points.appendleft(self.pos.int() if self.pos else None)
        
    def read(self):
        if DEVICE == "pc":
            return self.video_stream.read()
        return self.video_stream.capture_array()
    
    def _update(self) -> bool | None:
        frame = self.read()
        
        if frame is None:
            return
        
        frame = imutils.resize(frame, width=600)
        self.compute(frame)

        if self.preview:
            for i in range(1, len(self.points)):
                if self.points[i - 1] is None or self.points[i] is None:
                    continue
                
                thickness = int(np.sqrt(BUFFER_SIZE / float(i + 1)) * 2.5)
                cv2.line(frame, self.points[i - 1], self.points[i], BALL_TRACK_COLOUR, thickness)
            
            if self.pos and self.radius:
                cv2.circle(frame, self.pos.int(), int(self.radius), BALL_TRACK_COLOUR, 2)

                point2 = Vector(self.pos.x + self.velocity.x * 50, self.pos.y + self.velocity.y * 50)
                cv2.line(frame, self.pos.int(), point2.int(), (255, 0, 50), 3)

            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord("q"):
                return
        
        return True

    def set_update(self, event: Callable):
        self.update_events.append(event)

    def start(self) -> None:
        if DEVICE == "pi":
            self.video_stream.start()
        
        ticks = 1
        prev = perf_counter()
        while True:
            if self.preview:
                if (ticks % FPS_UPDATE) == 0:
                    now = perf_counter()
                    dt = now - prev
                    prev = now
                    fps = FPS_UPDATE / dt 

                    cv2.setWindowTitle(self.window_name, f"FPS: {fps}")

            if not self._update():
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
