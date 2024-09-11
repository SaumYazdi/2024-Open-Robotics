"""
Calibration control panel.
Run this file 'calibrate.py' and access the control panel at <ipv4-address>:8080
"""

from threading import Thread
from camera import Camera
from server import Server
from fan import Fan

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
    camera = Camera("Ball Detector", preview=False, draw_detections=True)
    camera.set_update(update)
    server = Server(__name__)

    fan = Fan()
    fan.on()

    threads = [
        Thread(target=server.start),
        Thread(target=camera.start),
    ]

    for thread in threads:
        thread.start()
