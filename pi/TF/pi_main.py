# Run on Raspberry Pi 5

from ball import BallDetector
from camera import PCCamera
import cv2 as cv
from os.path import join, dirname

def pathname(filename):
    return join(dirname(__file__), filename)
    
import sys
sys.path.insert(1, "/home/che0307/Documents/2024-Open-Robotics/pi/modules")

from fan_module import Fan
from camera_module import Camera

if __name__ == "__main__":
    fan = Fan()
    fan.on()

    camera_processing = PCCamera()

    camera = Camera()
    detector = BallDetector('transformed_frozen_inference_graph.pb', 'ssd_mobilenet_v1_balls.pbtxt')
    
    camera_processing.show(detector.detect_frame, camera=camera)
