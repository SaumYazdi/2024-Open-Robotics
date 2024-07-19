import cv2 as cv

from ball import BallDetector
from camera import Camera


if __name__ == "__main__":

    detector = BallDetector('transformed_frozen_inference_graph.pb', 'ssd_mobilenet_v1_balls.pbtxt')
    
    video_camera = Camera()
    video_camera.show(detector.detect_frame)

    # img = cv.imread('assets/example.jpg')
    # detector.detect_frame(img)