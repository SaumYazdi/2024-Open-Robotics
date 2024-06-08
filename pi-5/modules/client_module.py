import socket

class Client:
    def __init__(self, address: str, port: int):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((address, port))
    
    def send(self, message: str) -> str | None:
        self._socket.send(message.encode())
        
        data = self._socket.recv(1024)

        if data:
            return str(data.decode())
        else:
            return None