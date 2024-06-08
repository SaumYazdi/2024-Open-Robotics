# Run on computer
# Hub to connect to two other Raspberry Pi's.

from time import sleep
from modules.server_module import Server

server = Server("192.168.1.34", 8089)
server.start()