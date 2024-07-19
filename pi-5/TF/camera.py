import cv2 as cv
from cv2.typing import MatLike
from typing import Callable

class BBox:
    def __init__(self, left, top, right, bottom):
        self.left = left
        self.top = top
        self.right = right
        self.bottom = bottom

    @property
    def topleft(self):
        return (int(self.left), int(self.top))
    
    @property
    def bottomright(self):
        return (int(self.right), int(self.bottom))
    
    @property
    def size(self):
        return (int(self.right - self.left), int(self.bottom - self.top))

    @property
    def center(self):
        return (int(self.left + (self.right - self.left) / 2), int(self.top + (self.bottom - self.top) / 2))

class PCCamera:
    def __init__(self):
        self.video = cv.VideoCapture(0) 

    def draw_boxes(self, frame: MatLike, boxes: list):
        for bbox in boxes:
            left, top, right, bottom = bbox
            cv.rectangle(frame, (int(left), int(top)), (int(right), int(bottom)), (23, 230, 210), thickness=2)

    def get_largest_bbox(self, boxes: list) -> BBox:
        if len(boxes) == 0:
            return None
        sorted_boxes = sorted(boxes, key=lambda box: (box[0] - box[2]) * (box[1] - box[3]))
        return BBox(*sorted_boxes[-1])

    def get_angle(self, bbox):
        """
        Takes in the bounding box of a ball relative to the camera and returns the predicted angle.
        """
        center_x = bbox.center[0]
        # y = .105(x-331.8)
        return .105 * (center_x - 331.8)

    def update(self, frame: MatLike, boxes: list):
        # self.draw_boxes(frame, boxes)

        ball_bbox = self.get_largest_bbox(boxes)

        if ball_bbox == None:
            return
        
        cv.rectangle(frame, ball_bbox.topleft, ball_bbox.bottomright, (230, 40, 110), thickness=3)
        angle = self.get_angle(ball_bbox)

        font = cv.FONT_HERSHEY_SIMPLEX
        org = (15, 35)
        fontScale = 0.6
        color = (200, 40, 200) # BGR
        thickness = 2
        frame = cv.putText(frame, str(angle), org, font,  
                        fontScale, color, thickness, cv.LINE_AA) 

    def show(self, frame_process_function: Callable = None, frame: MatLike = None):
        while(True):
            if frame:
                ret, frame = self.video.read() 

            if frame_process_function:
                boxes = frame_process_function(frame)
                self.update(frame, boxes)
        
            cv.imshow('Camera', frame)
            
            if cv.waitKey(1) & 0xFF == ord('q'): 
                break
        
        self.video.release()
        cv.destroyAllWindows() 
