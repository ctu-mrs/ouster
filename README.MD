# This repository is archived

This repo was part of the old MRS UAV System and will be eventually deleted.

# Installation

Use the supplied installation script to install required dependencies.


# Change ouster to static IP:

1. Connect the sensor to an available Ethernet port on your host machine or router.

2. The sensor will automatically obtain an IP address either through link-local or DHCP (if preconfigured) depending on your network configuration.

3. If directly connecting to the host machine you may need to set your Ethernet interface to **Link-Local Only** mode. This can be done via the command line or GUI.

4. Once the sensor receives an IP address and is on the correct subnet we can talk to it using its hostname, `os-xxxxxxxxxxxxxx.local`, where `xxxxxxxxxxxxxx` is the sensor serial number. The sensor serial number can be found on a sticker affixed to the top of the sensor. There it shows the current IP of the sensor.

Other possibility is to get it with wireshark.

More can be found [here](https://static.ouster.dev/sensor-docs/image_route1/image_route2/connecting/connecting-to-sensors.html#connecting-to-sensor) and [here](https://static.ouster.dev/sensor-docs/image_route1/image_route2/networking_guide/networking_guide.html#linux).



Than all you need to do is to run this script to set the static IP.

```bash
sudo apt install httpie
echo '"10.10.20.90/24"' | http -v PUT http://169.254.13.27/api/v1/system/network/ipv4/override 
```
The first IP is the static one you are setting, the second one is the current ouster IP.
