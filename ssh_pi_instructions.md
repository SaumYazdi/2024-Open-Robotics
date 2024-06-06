# How to connect to your Pi's SSH

- Get list of devices (IP and MAC addresses)
  - `$ nmap -sn 192.168.1.0/24`

- Find `Raspberry Pi (Trading)` and copy the IP address

- Connect to Pi's SSH
  - `$ ssh <username>@<address>`

- It will prompt you on the first time connecting; enter "yes".

- Type the password you set for the Pi OS.

- You now have access to the Debian/Ubuntu shell on the Pi.