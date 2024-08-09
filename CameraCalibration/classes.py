from collections import deque
from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
from time import perf_counter
from typing import Callable

BUFFER_SIZE = 64
FPS_UPDATE = 120
MAX_BALL_JUMP = 720 # If ball is detected, how close the new pos must be for it the current position to update
BALL_TRACK_COLOUR = (0, 150, 255)
BALL_DIAMETER = 6.9 # centimetres
CALIBRATION_DISTANCE = 30

class Vector:
    """
    A 2D Vector defined by an x and y value.
    -> Vector(x, y)
    -> Vector((x, y))
    -> Vector([x, y])
    """
    _vec = [0, 0]
    def __init__(self, *args):
        if len(args) >= 2:
            self._vec = [args[0], args[1]]
        elif type(args[0]) in (list, tuple):
            self._vec = [args[0][0], args[0][1]]

    def __str__(self):
        return f"Vector<{self._vec[0]}, {self._vec[1]}>"

    def __repr__(self):
        return self._vec
    
    def __iter__(self):
        return iter(self._vec)
    
    def copy(self):
        return Vector(self._vec[0], self._vec[1])
    
    @property
    def x(self):
        return self._vec[0]

    @x.setter
    def x(self, value):
        self._vec[0] = value

    @property
    def y(self):
        return self._vec[1]

    @y.setter
    def y(self, value):
        self._vec[1] = value

    def int(self):
        return [int(_) for _ in self._vec]
    
def lerp(a: float, b: float, step: float = .4) -> float:
    return a + (b - a) * step

def dist_squared(a: Vector, b: Vector) -> float:
    return (a.x - b.x) ** 2 + (a.y - b.y) ** 2

class Camera:
    orange_lower = (1, 114, 245)
    orange_upper = (8, 203, 255)

    def __init__(self, window_name: str, preview: bool = True):
        self.points = deque(maxlen=BUFFER_SIZE)
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
        
        # only proceed if at least one contour was found
        if len(contours) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            pos = Vector(x, y)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
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
        
    def _update(self) -> bool | None:
        frame = self.video_stream.read()
        
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
        self.video_stream.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    camera = Camera("Ball Detector")
    camera.start()