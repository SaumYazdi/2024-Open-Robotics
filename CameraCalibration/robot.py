from camera import Camera
from math import degrees
import serial

"""
Usage:
    - Camera.get_distance(): Returns the ball's estimated distance
                                from the camera in centimetres.
    - Camera.get_angle():    Returns the angle of the ball relative
                                to the camera in radians.
"""

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
        
        self.ser = serial.Serial(RECEIVER, BAUD_RATE, timeout=1)
        self.ser.reset_input_buffer()
    
    def update(self):
        """System update loop. Updates the ball's distance and angle variables."""
        self.distance = camera.get_distance()
        self.angle = camera.get_angle()
        
        # self.send("test")
        # print(f"Distance: {str(self.distance): <16} Angle: {str(self.angle): <16}")
        
    def send(self, value: str | bytes):
        """
        Send serial data to the main controller.
        """
        if type(value) == str:
            value = value.encode("utf-8")
            
        self.ser.write(value + b'\n')
    
    def start(self):
        """
        Start reading the camera and producing ball data.
        """
        self.camera.start()
        

if __name__ == "__main__":
    camera = Camera("Soccer Robot", preview=False, draw_detections=False)
    
    robot = Robot(camera)
    robot.start()
