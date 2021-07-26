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

* **IP** = 192.168.4.1
* **username** = pi
* **password** = admin

From a windows client you can use [PuTTY](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html).

From a linux client you can use `ssh pi@192.168.4.1`.

## Ethernet Interface
This interface is used by the LiDAR LMS111. It is configured to be static and thus it does not provide internet access even if an ethernet cable is connected.

If you require to change the configuration of this interface, check the file located at: `/etc/network/interfaces`. For the purpose of clarification, this interface can be used in two ways:

* **static**: To retrieve data from the LiDAR or other static purposes. The address `169.254.97.100` was chosen to match the LiDAR static configuration.
```
auto eth0
iface eth0 inet static
        address 169.254.97.100
        netmask 255.255.0.0
```
* **dhcp**: To obtain internet access.
```
auto eth0
allow-hotplug eth0
iface eth0 inet dhcp
```

It is worth mentioning that only one this option can be used. This means that one of these code block needs to be commented with "#" while the other one is active.

In some cases, this interface will not automatically detect when an ethernet cable is plugged in. This can be easily solved by restarting the interface with `sudo ifdown eth0` and then `sudo ifup eth0`.

## Internet access
Even though the wired and wireless interfaces do not provide internet access, it is possible to do this by employing several alternatives.

* **USB WiFi Dongle**: A dongle can be purchased and pluged into the raspberry usb ports to provide an additional wireless interface. This method might required specific configuration and drivers.
* **USB to Ethernet Adapter**: An adapter can be pluged into the raspberry usb ports to provide an additional ethernet interface. This method might require additional configuration.
* **USB Tethering**: The raspberry is already configured to obtain internet access when a cellphone is connected to one of the usb ports. The usb tethering option needs to be active in the phone. This configuration was added in the file located at: `/etc/network/interfaces`. The following lines were used:
```
allow-hotplug usb0
iface usb0 inet dhcp
```
