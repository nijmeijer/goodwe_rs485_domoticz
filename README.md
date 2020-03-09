Domoticz python plugin for Goodwe Solar Inverters via RS485

Based on C-code by https://github.com/jantenhove
Converted to python for Domoticz

Status
------
Under development. Requires domoticz hardware-timeout to be set to 10 min. 

Prerequesites
-------------
An RS485 cable to the computer that runs domoticz (see file goodwe_rs485_to_usb_cable.JPG)
In my goodwe inverter, the RS485 connector is located under the WiFi Dongle. This must be removed.
I am not sure if the RS485 interface operates, when the Wifi Dongle is connected.

Limitations
------------
Currently only a single Inverter is supported (since I own just one)
However, multiple inverters can be connected to a single RS485 bus. 

Install
-----------
in folder domoticz/plugins  :
git clone https://github.com/nijmeijer/goodwe_rs485_domoticz goodwe_rs485
This will create a folder named goodwe_rs485, containing the file "plugin.py" as expected by domoticz

Restart domoticz, and add the new HARDWARE.

Add the devices that you are interested in. Currently, only Momentary Power (Pac in W) and Total Energy (generated by the inverter, in Wh) are supported. 
However, the inverter also returns Iac, Fac, HoursOfOperation, Temperature, mode of operation. These are not handled by the plugin (yet).



