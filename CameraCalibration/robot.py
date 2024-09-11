"""
Main camera systems which will parse the ball location data to the main
"""

from camera import Camera
from math import degrees
import serial
import struct

RECEIVER = "/dev/ttyACM0" # Work out the name for PICO
BAUD_RATE = 9600
        
class Robot:
    """
    System to parse ball data serially to the main controller.
    """
    distance = None
    angle = None
    def __init__(self, camera: Camera):
        self.camera = camera
        self.camera.set_update(self.update)
        
        self.tick = 0
    
    def update(self):
        """System update loop. Updates the ball's distance and angle variables."""
        self.distance = self.camera.get_distance()
        self.angle = self.camera.get_angle()
        
        try:
            self.ser = serial.Serial(RECEIVER, BAUD_RATE, timeout=3)
        except serial.serialutil.SerialException:
            # BISMILLAH DONT THROW ERROR
            self.ser = None
            pass

        if type(self.ser) != serial.Serial:
            return
        
        if self.distance and self.angle:
            self.send(self.distance, -degrees(self.angle))
        # print(f"Distance: {str(self.distance): <16} Angle: {str(self.angle): <16}")
        
        self.tick += 1
        if self.tick % 5 == 0:
            self.ser.reset_input_buffer()
        
    def send(self, dist, angle):
        """
        Send serial data to the main controller.
        """
        send_data = struct.pack('f', dist) + struct.pack('f', angle)
            
        self.ser.write(send_data)
    
    def start(self):
        """
        Start reading the camera and producing ball data.
        """
        self.camera.start()
        print("e")
        

if __name__ == "__main__":
    camera = Camera("Soccer Robot", preview=False, draw_detections=False)
    
    robot = Robot(camera)
    robot.start()
