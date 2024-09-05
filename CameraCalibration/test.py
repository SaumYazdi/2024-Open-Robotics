"""
Main camera systems which will parse the ball location data to the main
"""

# from camera import Camera
from math import degrees
import serial
import struct
import time

RECEIVER = "/dev/ttyACM0" # Work out the name for PICO
BAUD_RATE = 9600
        
class Robot:
    """
    System to parse ball data serially to the main controller.
    """
    distance = None
    angle = None
    ticks = 0
    def __init__(self):
        # self.camera = camera
        # self.camera.set_update(self.update)
        
        self.ser = serial.Serial(RECEIVER, BAUD_RATE, timeout=1)
        self.ser.reset_input_buffer()
    
    def update(self):
        """System update loop. Updates the ball's distance and angle variables."""
        # self.distance = camera.get_distance()
        # self.angle = camera.get_angle()
        
        value = self.ticks * .001
        self.send(value)
        # print(f"Distance: {str(self.distance): <16} Angle: {str(self.angle): <16}")
        
        self.ticks += 1
        
    def send(self, value: str | bytes | float):
        """
        Send serial data to the main controller.
        """
        if type(value) == str:
            value = value.encode("utf-8")
            
        elif type(value) == float:
            send_data = struct.pack("f", value)
        
        self.ser.write(send_data)
        print(f"Value: {value}, {send_data}, {send_data.hex()}")
    
    def start(self):
        """
        Start reading the camera and producing ball data.
        """
        # self.camera.start()
        while True:
            self.update()
            time.sleep(0.1)
        

if __name__ == "__main__":
    # camera = Camera("Soccer Robot", preview=False, draw_detections=False)
    
    robot = Robot()
    robot.start()
