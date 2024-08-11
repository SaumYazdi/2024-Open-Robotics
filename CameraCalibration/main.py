from threading import Thread
from camera import Camera
from server import Server

def update():
    """
    Update server values with ball measurements for calibration
    """
    if camera.pos is not None:
        server.x_offset = camera.pos.x
    else:
        server.x_offset = None
    server.radius = camera.radius
    if server.lower and server.upper:
        camera.set_color_bounds(server.lower, server.upper)
        server.lower = server.upper = None
        
    if server.show_preview:
        server.preview = camera.image.copy()

if __name__ == "__main__":
    camera = Camera("Ball Detector", preview=False, draw_detections=True)
    camera.set_update(update)
    server = Server(__name__)

    threads = [
        Thread(target=server.start),
        Thread(target=camera.start),
    ]

    for thread in threads:
        thread.start()
