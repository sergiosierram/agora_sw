# Network Information
This file contains useful information about the network interfaces of the AGoRA Walker raspberry.

## Wireless Interface

### Configuration
It is configured as a bridged wireless access point and it generates a wireless network with the following main characteristics. 

* **interface** = wlan0
* **ssid** = AGoRA_Walker
* **password** = walker1234
* **hardware mode** = IEEE 802.11g (2.4 GHz)
* **routing and ip masquerading** (internet access trough ethernet) = enabled

If you require to change the access point configuration, check the file located at: `/etc/hostapd/hostapd.conf`. You might need to restart the interface so that the changes take effect. This can be done by rebooting the raspberry with `sudo systemctl reboot` or restarting the interface with `sudo ifdown wlan0` and then `sudo ifup wlan0`.

Please check the documentation at [raspberrypi.org](https://www.raspberrypi.org/documentation/configuration/wireless/access-point-routed.md) for further details.

### Login trough SSH
The AGoRA Walker raspberry has SSH enabled to provide remote access from other computers. Once you have connected to the wireless network, you can login with the following credentials:

* IP = 192.168.4.1
* username = pi
* password = admin

From a windows client you can use [PuTTY](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html).

From a linux client you can use `ssh pi@192.168.4.1`.

## Ethernet Interface
This interface works normally and provides internet access to the wireless access point. In some cases, this interface will not automatically detect when an ethernet cable is plugged in. This can be easily solved by restarting the interface with `sudo ifdown eth0` and then `sudo ifup eth0`.
