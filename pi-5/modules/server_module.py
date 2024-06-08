import socket
from threading import Thread

class Server:
    def __init__(self, address: str, port: int):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.bind((address, port))
        self._socket.listen(2)

    def _run(self):
        while True:
            connection, address = self._socket.accept()
            buffer = connection.recv(64)

            if len(buffer) > 0:
                print(buffer.decode())

    def start(self):
        self._thread = Thread(target=self._run)
        self._thread.start()