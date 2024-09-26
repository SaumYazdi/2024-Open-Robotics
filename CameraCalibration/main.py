"""
Testing interface
Run this file 'main.py' and access the control panel at <ipv4-address>:8080
"""

from threading import Thread
<<<<<<< HEAD
from camera import FrontFacingCamera, DownFacingCamera
=======
from camera import DownFacingCamera
>>>>>>> 652d6e8afdbefdaa926e39a9499bf91f1bd98484
from server import Server
from robot import Robot
from fan import Fan
import sys

def update():
    """
    Update server values with ball measurements for calibration
    """
    if server is None:
        return
        
    if camera.pos is not None:
        server.x_offset = camera.pos.x
    else:
        server.x_offset = None
        
    server.radius = camera.radius
    server.radial_distance = camera.radial_distance
    server.angle = camera.angle
    server.distance = camera.distance

    camera.mask_radius = server.mask_radius
    
    if server.lower != camera.orange_lower or server.upper != camera.orange_upper:
        camera.set_color_bounds(server.lower, server.upper)
    
    server.fps = camera.fps
    if server.show_preview:
        server.preview = camera.image.copy()

if __name__ == "__main__":
<<<<<<< HEAD
    camera = DownFacingCamera("360", preview=False, draw_detections=False, camera_port=0, detect_back=True)
    camera2 = DownFacingCamera("offset", preview=False, draw_detections=False, camera_port=1)
=======
    camera = DownFacingCamera("Soccer Robot", preview=False, draw_detections=False)
>>>>>>> 652d6e8afdbefdaa926e39a9499bf91f1bd98484
    camera.set_update(update)
    
    robot = Robot(camera, camera2)
    
    fan = Fan()
    fan.on()
 
    threads = [
        Thread(target=camera.start),
        Thread(target=camera2.start),
    ]

    server = None
    args = sys.argv
    if len(args) > 1:
        arg = args[1]
    
        if arg == "view":
            print("Starting flask webserver")
            camera.draw_detections = True
            server = Server(__name__)
            threads.append(Thread(target=server.start))
           
    for thread in threads:
        thread.start()
