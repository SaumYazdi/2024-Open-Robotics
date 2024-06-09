import socket
from threading import Thread
from modules.settings import get_setting
from os.path import join, dirname
import numpy as np
import cv2
import base64

START_ARRAY = b'\xff' # 255
END_ARRAY = b'\xfe' # 254

class Server:
    def __init__(self, address: str, port: int):
        self._address = address
        self._port = port
        
        self.connections = {}

        self.arr = None

    def image_from_bytes(self, img_str: bytes):
        with open(join(dirname(__file__), "test.txt"), "wb") as f:
            f.write(img_str)
    
    def show(self):
        with open(join(dirname(__file__), "test.txt"), "rb") as f:
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

            elif data[-1:] == END_ARRAY: # If last byte is '\xfe'
                if self.arr:
                    print("Finished array.")

                    recevied_arr = self.arr[:-1] # Strip last 'end array' byte.
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
