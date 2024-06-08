

from time import sleep
from modules.fan_module import Fan

fan = Fan()
fan.on()

sleep(3)

fan.off()

print(fan.get())