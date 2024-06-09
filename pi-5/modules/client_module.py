import socket

from numpy import ndarray

class Client:
    def __init__(self, address: str, port: int):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((address, port))
    
    def send(self, message: str) -> str | None:
        if type(message) == str:
            self._socket.send(message.encode())
            
        elif type(message) == ndarray:
            message_bytes = message.tobytes()
            self._socket.send(message_bytes)
        
        else:
            return
        
        data = self._socket.recv(1024)

        if data:
            return str(data.decode())
        else:
            return None
