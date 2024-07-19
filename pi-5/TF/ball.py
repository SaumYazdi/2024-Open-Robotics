import cv2 as cv
from cv2.typing import MatLike

class BallDetector:
    def __init__(self, pb_path: str, pbtxt_path: str, threshold=0.28):
        self.cvNet = cv.dnn.readNetFromTensorflow(pb_path, pbtxt_path)

        self.threshold = threshold
        self.previous_position = None

        # focused_area
        # focused_pos
        
    def detect_frame(self, img: MatLike) -> list[list]:
        rows = img.shape[0]
        cols = img.shape[1]

        self.cvNet.setInput(cv.dnn.blobFromImage(img, scalefactor=1.0/127.5, size=(300, 300), mean=(127.5, 127.5, 127.5), swapRB=True, crop=False))
        cvOut = self.cvNet.forward()

        boxes = []

        for detection in cvOut[0,0,:,:]:
            score = float(detection[2])
            if score > self.threshold:
                left = detection[3] * cols
                top = detection[4] * rows
                right = detection[5] * cols
                bottom = detection[6] * rows

                boxes.append([left, top, right, bottom])

        return boxes