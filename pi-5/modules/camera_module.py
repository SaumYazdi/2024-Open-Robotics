#!/usr/bin/python3

from picamera2 import Picamera2
from libcamera import Transform, ColorSpace

import cv2
from numpy import ndarray

class Camera:
    def __init__(self):
        self._camera = Picamera2()
        
        config = self._camera.create_still_configuration(
            main={"format": 'XRGB8888', "size": (640, 480)}, 
            lores={"size": (320, 240)}, display="lores"
        )
        
        self._camera.configure(config)
    
    def screenshot(self, filename: str):
        self._camera.start()
        self._camera.capture_file(filename)
        self._camera.close()
        
    def get_ss_array(self) -> ndarray:
        self._camera.start()
        im = self._camera.capture_array()
        self._camera.close()
        return im
        
    def ball_detection_test(self):
        self._camera.start()
            
        while True:
            im = self._camera.capture_array()
            
            hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv,(10, 100, 20), (25, 255, 255) )

            cv2.imshow("Orange", mask)
            cv2.waitKey(1)

    def face_detection_test(self):
        face_detector = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")
        cv2.startWindowThread()

        self._camera.start()

        while True:
            im = self._camera.capture_array()

            grey = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
            faces = face_detector.detectMultiScale(grey, 1.1, 5)

            for (x, y, w, h) in faces:
                cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0))

            cv2.imshow("Camera", im)
            cv2.waitKey(1)

if __name__ == "__main__":
    camera = Camera()
    # camera.face_detection_test()
    camera.ball_detection_test()
    # print(camera.get_ss_array())
