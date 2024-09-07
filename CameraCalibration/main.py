"""
Testing interface
Run this file 'main.py' and access the control panel at <ipv4-address>:8080
"""

from threading import Thread
from camera import Camera
from server import Server
from robot import Robot
from fan import Fan

def update():
    """
    Update server values with ball measurements for calibration
    """
    if camera.pos is not None:
        server.x_offset = camera.pos.x
    else:
        server.x_offset = None
        
    server.radius = camera.radius
    server.radial_distance = camera.radial_distance
    server.angle = camera.angle
    
    if server.lower and server.upper:
        camera.set_color_bounds(server.lower, server.upper)
        server.lower = server.upper = None
    
    server.fps = camera.fps
    if server.show_preview:
        server.preview = camera.image.copy()

if __name__ == "__main__":
    camera = Camera("Soccer Robot", preview=False, draw_detections=False)
    camera.set_update(update)
    server = Server(__name__)
    
    robot = Robot(camera)
    
    fan = Fan()
    fan.on()

    threads = [
        # Thread(target=server.start),
        Thread(target=robot.start),
    ]

    for thread in threads:
        thread.start()
