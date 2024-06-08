# Allow user input in client to send to server

from modules.client_module import Client

client = Client("192.168.1.34", 8089)

while True:
    message = str(input("> "))

    if not message:
        break

    resp = client.send(message)
    print(f'Received "{resp}"')

