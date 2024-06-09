#!/usr/bin/python3

from picamera2 import Picamera2
from libcamera import Transform, ColorSpace

from settings import get_setting

import cv2
from numpy import ndarray

class Camera:
    def __init__(self):
        self._camera = Picamera2()
        
        resolution = get_setting("resolution")
        
        config = self._camera.create_still_configuration(
            main={"format": 'XRGB8888', "size": resolution}
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
        
    def ball_detection_test(self, win_name = "Orange"):
        self._camera.start()
            
        while cv2.waitKey(1) == -1:
            im = self._camera.capture_array()
            
            hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv,(10, 100, 20), (25, 255, 255) )

            cv2.imshow(win_name, mask)
            
        print("Key pressed, exiting..")
        cv2.destroyWindow(win_name)

    def face_detection_test(self, win_name = "Face Detection"):
        face_detector = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")
        cv2.startWindowThread()

        self._camera.start()

        while cv2.waitKey(1) == -1:
            im = self._camera.capture_array()

            grey = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
            faces = face_detector.detectMultiScale(grey, 1.1, 5)

            for (x, y, w, h) in faces:
                cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0))

            cv2.imshow(win_name, im)
            
        print("Key pressed, exiting..")
        cv2.destroyWindow(win_name)

if __name__ == "__main__":
    camera = Camera()
    # camera.face_detection_test()
    camera.ball_detection_test()
    # print(camera.get_ss_array())
