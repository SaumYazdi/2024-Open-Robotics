#!/usr/bin/python3

from picamera2 import Picamera2
from libcamera import Transform, ColorSpace

import cv2

class Camera:
    def __init__(self):
        self._camera = Picamera2()
        
        config = self._camera.create_still_configuration(
            main={"format": 'XRGB8888', "size": (640, 480)}, 
            lores={"size": (320, 240)}, display="lores"
        )
        
        self._camera.configure(config)
        
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
    camera.face_detection_test()
