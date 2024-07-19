from settings import get_path
from threading import Thread
import keyboard
import numpy as np
import socket
import sys

ADDRESS = "10.173.49.142"
PORT = 8089
CHUNK_SIZE = 8

def server():
    print("Started Server")

    server_socket.bind((ADDRESS, PORT))
    server_socket.listen(5)

    print(f"Listening on {ADDRESS}")

    while True:
        try:
            connection, address = server_socket.accept()
        except OSError:
            break

        data = connection.recv(CHUNK_SIZE)
        print(f"Received {data.decode()}")

    server_socket.close()

def client():
    print("Started Client")

    client_socket.connect((ADDRESS, PORT))

    print(f"Connected on {ADDRESS}")

    print("Sent 'test'")
    client_socket.send("test".encode())

i = 0
def send_chunk():
    global i

    print(i)

    start = i * CHUNK_SIZE
    end = (i + 1) * CHUNK_SIZE
    print(img_bytes[start : end])

    if i >= 5:
        server_socket.close()
        client_socket.close()
        sys.exit()

    i += 1

if __name__ == "__main__":

    with open(get_path("test.jpg"), "rb") as f:
        img_bytes = f.read()
        f.close()

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    Thread(target=server).start()
    Thread(target=client).start()

    keyboard.add_hotkey("esc", send_chunk)