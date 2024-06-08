# Run on computer
# Hub to connect to two other Raspberry Pi's.

from time import sleep
from modules.server_module import Server
from modules.settings import get_setting

server = Server(get_setting("ipv4"), get_setting("port"))
server.start()