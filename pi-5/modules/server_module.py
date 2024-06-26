import socket
from threading import Thread
import numpy as np

import cv2
import base64

from modules.settings import get_setting, get_path

START_ARRAY = b'\xff\xff\xff'
END_ARRAY = b'\xfe\xfe\xfe'

class Server:
    def __init__(self, address: str, port: int):
        self._address = address
        self._port = port
        
        self.connections = {}

        self.arr = None

    def image_from_bytes(self, img_str: bytes):
        with open(get_path("test.txt"), "wb") as f:
            f.write(img_str)
    
    def show(self):
        with open(get_path("test.txt"), "rb") as f:
            data = f.read()

            orig = base64.b64decode(data)
            np_arr = np.frombuffer(orig, dtype=np.uint8)
            img = cv2.imdecode(np_arr, flags=1)
            cv2.imwrite("show.jpg", img)


    def on_new_connection(self, connection, address):
        print(f"Connected with {address}")
        
        while True:
            data = connection.recv(1024)
            
            if not data:
                break

            if data == "show".encode():
                self.show()
                break

            if self.arr is not None:
                self.arr += data
            
            if data == START_ARRAY:
                print("Receiving array..")

                self.arr = b''

            elif data[-3:] == END_ARRAY: # If last three byte
                if self.arr:
                    print("Finished array.")

                    recevied_arr = self.arr[:-3] # Strip last 'end array' byte.
                    self.image_from_bytes(recevied_arr)

                    self.arr = None

                else:
                    raise Exception("Did not receive starting byte for array.")
                
            else:
                if self.arr == None:
                    print(address, ">>", data.decode())
                    connection.send(data)
        
        print(f"{address} disconnected.")

    def _run(self):
        print(f"Starting on {self._address}")

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.bind((self._address, self._port))
        self._socket.listen(5)
        
        while True:
            connection, address = self._socket.accept()
            
            thread = Thread(target=self.on_new_connection, args=(connection, address,))
            thread.start()
            self.connections[address] = thread
        
        
    def start(self):
        self._thread = Thread(target=self._run)
        self._thread.start()
