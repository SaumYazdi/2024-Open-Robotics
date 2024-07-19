import cv2 as cv

from ball import BallDetector
from camera import PCCamera

if __name__ == "__main__":

    detector = BallDetector('transformed_frozen_inference_graph.pb', 'ssd_mobilenet_v1_balls.pbtxt')
    
    video_camera = PCCamera()
    video_camera.show(detector.detect_frame)

    # img = cv.imread('assets/example.jpg')
    # detector.detect_frame(img)
    
    # $ env/bin/python pi-5/TF/main.py
