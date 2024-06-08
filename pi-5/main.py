# Run on Raspberry Pi (5)

from modules.client_module import Client
from modules.fan_module import Fan

if __name__ == "__main__":
    fan = Fan()
    fan.on()

    client = Client("192.168.1.34", 8089)

    while True:
        message = str(input("> "))

        if not message:
            break

        resp = client.send(message)
        print(f'Received "{resp}"')

