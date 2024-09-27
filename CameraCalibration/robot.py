"""
Main camera systems which will parse the ball location data to the main
"""

from camera import DownFacingCamera
from math import degrees, pi
import serial
import struct

# RECEIVER = "/dev/ttyACM0" # USB
RECEIVER = "/dev/ttyAMA0" # GPIO
BAUD_RATE = 115200

END_SEQUENCE = b'\xab\xcd'

class Robot:
    """
    System to parse ball data serially to the main controller.
    """
    distance = None
    angle = None
    def __init__(self, camera1: DownFacingCamera, camera2: DownFacingCamera = None):
        self.camera1 = camera1
        self.camera2 = camera2

        self.camera1.set_update(self.update)
        
        self.tick = 0
    
    def update(self):
        """System update loop. Updates the ball's distance and angle variables."""

        # Prioritise results from down facing camera (PORT 1)
        self.distance = self.camera1.get_distance()
        self.angle = self.camera1.get_angle()
        if not (self.distance and self.angle) and self.camera2 is not None:
            self.distance = self.camera2.get_distance()
            # self.angle = self.camera2.get_angle()
        
            # OTHER SOLUTION (TWO down-facing cameras, the second one is oriented 90 deg another way so have to correct for it.)
            self.angle = self.camera2.get_angle()
            if self.angle is not None:
                self.angle += pi / 2
        try:
            self.ser = serial.Serial(RECEIVER, BAUD_RATE, timeout=1)
        except serial.serialutil.SerialException:
            # BISMILLAH DONT THROW ERROR
            self.ser = None
            print("Pico Not Connected")
            pass

        print(self.distance, self.angle)
        if type(self.ser) != serial.Serial:
            return
        
        if self.distance and self.angle:
            self.send(self.distance, -degrees(self.angle))
        
        self.tick += 1
        if self.tick % 4 == 0:
            self.ser.reset_input_buffer()
        
    def send(self, dist, angle):
        """
        Send serial data to the main controller.
        """
        send_data = struct.pack('f', dist) + struct.pack('f', angle) + END_SEQUENCE
            
        self.ser.write(send_data)
    
    def start(self):
        """
        Start reading the camera and producing ball data.
        """
        self.camera1.start()
        

if __name__ == "__main__":
    camera1 = DownFacingCamera("FrontBack", preview=False, draw_detections=False, camera_port=0, detect_back=True)
    camera2 = DownFacingCamera("SideToSide", preview=False, draw_detections=False, camera_port=1)
    
    robot = Robot(camera1, camera2)
    robot.start()
