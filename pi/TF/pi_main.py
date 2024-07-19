# Run on Raspberry Pi 5

from ball import BallDetector
from camera import PCCamera
import cv2 as cv

from modules.fan_module import Fan
from modules.camera_module import Camera

if __name__ == "__main__":
    fan = Fan()
    fan.on()

    camera_processing = PCCamera(active=False)

    camera = Camera()
    detector = BallDetector('transformed_frozen_inference_graph.pb', 'ssd_mobilenet_v1_balls.pbtxt')
    
    camera_processing.show(detector.detect_frame, camera=camera)
