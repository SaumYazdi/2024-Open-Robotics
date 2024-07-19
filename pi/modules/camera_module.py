#!/usr/bin/python3

machine_type = "pi"

try:
    from picamera2 import Picamera2
    from libcamera import Transform, ColorSpace
except ModuleNotFoundError:
    machine_type = "pc"

if machine_type == "pi":
    from modules.settings import get_setting, get_path
elif machine_type == "pc":
    from settings import get_setting, get_path

import cv2
from cv2.typing import MatLike
from numpy import ndarray

class Camera:
    def __init__(self):
        if machine_type == "pi":
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
        
    def read(self) -> MatLike:
        arr = self.get_ss_array()
        return MatLike(arr)

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

    def pc_camera_test(self, image_name: str, win_name="Ball"):
        im = cv2.imread(get_path(image_name))

        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        # hsv = cv2.cvtColor(im, cv2.COLOR_RGB2HSV)
        # mask = cv2.inRange(hsv,(10, 80, 20), (20, 240, 240) )
        mask = cv2.inRange(hsv,(10, 100, 20), (25, 255, 255) )

        light_orange = (1, 190, 200)
        dark_orange = (18, 255, 255)
        # mask = cv2.inRange(im, light_orange, dark_orange)
        # mask = cv2.inRange(im, (223, 67, 27), (243, 100, 34))
        # mask = cv2.inRange(hsv, (255, 141, 14), (74, 209, 7))

        while cv2.waitKey(1) == -1:
            cv2.imshow(win_name, im)
            
        while cv2.waitKey(1) == -1:
            cv2.imshow(win_name, mask)

        result = cv2.bitwise_and(im, im, mask=mask)
            
        while cv2.waitKey(1) == -1:
            cv2.imshow(win_name, result)

        print("Key pressed, exiting..")
        cv2.destroyWindow(win_name)



if __name__ == "__main__":
    camera = Camera()
    # camera.face_detection_test()
    # camera.ball_detection_test()
    # print(camera.get_ss_array())
    camera.pc_camera_test("test.jpg")
