import socket

from numpy import ndarray
from cv2 import imencode

START_ARRAY = b'\xff\xff\xff'
END_ARRAY = b'\xfe\xfe\xfe'

class Client:
    def __init__(self, address: str, port: int):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((address, port))
    
    def send_array(self, image: ndarray):
        success, encoded_image = imencode(".jpg", image)
        image_bytes = encoded_image.tobytes()
        
        self._socket.send(START_ARRAY)
        self._socket.send(image_bytes)
        self._socket.send(END_ARRAY)
    
    def send(self, message: str) -> str | None:
        if type(message) == str:
            self._socket.send(message.encode())
        else:
            raise Exception("Client.send() expecting a <str>.")
        
        data = self._socket.recv(1024)

        if data:
            return str(data.decode())
        else:
            return None
