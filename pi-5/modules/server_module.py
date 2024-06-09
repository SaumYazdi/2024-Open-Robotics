import socket
from threading import Thread

class Server:
    def __init__(self, address: str, port: int):
        self._address = address
        self._port = port
        
        self.connections = {}

    def on_new_connection(self, connection, address):
        print(f"Connected with {address}")
        
        while True:
            data = connection.recv(1024)
            
            if not data:
                break

            print(address, ">>", data.decode())
            connection.send(data)
        
        print(f"{address} disconnected.")

    def _run(self):
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
        
if __name__ == "__main__":
    
    server = Server("192.168.1.34", 8089)
    server.start()
