from time import sleep
from modules.fan_module import Fan
from modules.server_module import Server

fan = Fan()
fan.on()

server = Server("192.168.1.34", 8089)
server.start()