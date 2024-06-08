# Run on Raspberry Pi (5)

from modules.client_module import Client
from modules.fan_module import Fan
from modules.settings import get_setting

if __name__ == "__main__":
    fan = Fan()
    fan.on()

    client = Client(get_setting("ipv4"), get_setting("port"))

    while True:
        message = str(input("> "))

        if not message:
            break

        resp = client.send(message)
        print(f'Received "{resp}"')

