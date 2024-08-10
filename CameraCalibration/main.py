from threading import Thread
from camera import Camera
from server import Server

def update():
    server.radius = camera.radius
    if server.show_preview:
        server.preview = camera.image

if __name__ == "__main__":
    camera = Camera("Shibal", preview=True)
    camera.set_update(update)
    server = Server(__name__)

    threads = [
        Thread(target=server.start),
        Thread(target=camera.start),
    ]

    for thread in threads:
        thread.start()
