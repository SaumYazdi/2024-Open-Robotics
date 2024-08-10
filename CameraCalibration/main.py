from threading import Thread
from camera import Camera
from server import Server

def update():
    if camera.radius:
        server.radius = camera.radius
    server.preview = camera.image

if __name__ == "__main__":
    camera = Camera("Shibal", preview=False)
    camera.set_update(update)
    server = Server(__name__)

    threads = [
        Thread(target=server.start),
        Thread(target=camera.start),
    ]

    for thread in threads:
        thread.start()
