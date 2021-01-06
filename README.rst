# Installation

Use the supplied installation script to install required dependencies.

# Change ouster to static IP:

```bash
sudo apt install httpie
echo '"10.10.20.90/24"' | http -v PUT http://169.254.13.27/api/v1/system/network/ipv4/override 
```
The first IP is the static one you are setting, the second one is the current ouster IP (get it with wireshark).
