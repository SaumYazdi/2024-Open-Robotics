from threading import Thread
from classes import Camera
from server import Server

# def update():
#     print(camera.pos)

if __name__ == "__main__":
    camera = Camera("Shibal", preview=False)
    # camera.set_update(update)
    server = Server(__name__)

    threads = [
        Thread(target=server.run),
        Thread(target=camera.start),
    ]

    for thread in threads:
        thread.start()