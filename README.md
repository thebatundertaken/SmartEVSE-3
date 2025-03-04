SmartEVSE v3
=========
[![GitHub release](https://img.shields.io/github/release/thebatundertaken/SmartEVSE-3.svg)](https://github.com/thebatundertaken/SmartEVSE-3/releases)

# Changes to original firmware v3.0.0
- Adaptation for a 3 phases installation with 1 phase fixed cable or socket
- New solar boost feature to allow charging over max mains when solar power surplus available
- Device screen on Smart/Solar mode renders decimal amps for phases consumption
- Added scheduled charging with switch on and off time (operating hours)
- New web status page (UI design, Rest API and no webSockets)
- New max EVSE temperature menu and settings
- New RGB leds enabled/disabled menu and settings
- Support for API meter readings (Sensorbox v2)
- Massive code refactor to C++ objects
- Improved timers and tasks
- Special thanks to **Serkri** for his work (https://github.com/serkri/SmartEVSE-3)

![Status page](./pictures/statuspage.jpg)

![Status page](./pictures/statuspage-mobile.jpg)

![Status page](./pictures/power-monitor.jpg)

![Status page](./pictures/device.jpg)

$~$
# Menu options and configuration

- [Device configuration and all menu options explained](docs/configuration.md)<br>

- [Official SmartEVSE 3 manual](manual/SmartEVSEv3_install_v3.0.pdf)<br>

$~$
# How to install the hardware

- [3 phases mains with 1 phase charger wiring diagrama and electrical installation](docs/installation.md)


<br>

$~$
# Setting up WiFi

> Documentation from: https://github.com/tzapu/WiFiManager

- From SmartEVSE menu select `WIFI` option and `SetupWifi` suboption
- In 5 seconds Wifi portal will start and SmartEVSE will display Wifi access point name and password
- using any wifi enabled device with a browser (computer, phone, tablet) connect to the newly created Access Point
- because of the Captive Portal and the DNS server you will either get a 'Join to network' type of popup or get any domain you try to access redirected to the configuration portal
- choose one of the access points scanned, enter password, click save
- SmartEVSE will try to connect. If successful, it relinquishes control back to your app. If not, reconnect to AP and reconfigure.

- When your SmartEVSE starts up, it sets it up in Station mode and tries to connect to a previously saved Access Point
- if this is unsuccessful (or no previous network saved) it moves the SmartEVSE into Access Point mode and spins up a DNS and WebServer (default ip 192.168.4.1)

![ESP8266 WiFi Captive Portal Homepage](http://i.imgur.com/YPvW9eql.png) ![ESP8266 WiFi Captive Portal Configuration](http://i.imgur.com/oicWJ4gl.png)


$~$
# Building the firmware
* Install platformio-core https://docs.platformio.org/en/latest/core/installation/methods/index.html
* Compile `firmware.bin`: platformio run
* Compile `spiffs.bin`: platformio run -t buildfs

$~$
## Updating firmware on the device
> [!IMPORTANT]
> **You need to flash both files `firmware.bin` and `spiffs.bin` to fully update firmware**.

File `firmware.bin` contains C++ compiled code and file `spiffs.bin` contains embeded webserver HTML content.

> [!CAUTION]
> YOU CANNOT FLASH THE DEVICE WITH ANOTHER FILENAME rather than `firmware.bin` and `spiffs.bin`!


### Option 1: WiFi flashing
* Configure SmartEVSE WiFi
* Open device `/update` endpoint on browser without HTTPS *(ex: http://your-smartevse-address/update)*
* Select the `firmware.bin` archive
* After OK, select `spiffs.bin`
* If you get FAIL, check your wifi connection and try again
* After OK, wait 10-30 seconds and your new firmware including the webserver should be online!
* Repeat same steps with `spiffs.bin` archive


$~$
### Option 2: USB flashing
* **Windows users**: Install driver for Virtual Port https://www.silabs.com/documents/public/software/CP210x_VCP_Windows.zip
* Open `platformio.ini` file and replace COM4 port with your SmartEVSE device port *(check via Windows device manager, Linux users: the device will present itself usually as /dev/ttyUSB0)*

You can use the following flashing software:

  1. **PlatformIO**:
     ```
     pio run -t upload
     pio run -t uploadfs
     ```

     THIS IS THE PREFERRED WAY, because it also flashes your bootloader and the partitions.bin; so whatever you messed up, this will fix it!

  2. **ESPTool**:
     ```
     sudo apt install esptool
     esptool --port /dev/ttyUSB0 write_flash 0x10000 firmware.bin
     esptool --port /dev/ttyUSB0 write_flash 0x1c0000 firmware.bin 
     ```
  3. Flash it with a **3rd party tool**:
     A nice 3rd party tool can be found here: https://github.com/marcelstoer/nodemcu-pyflasher
     Follow the instructions in the screenshot posted here: https://github.com/dingo35/SmartEVSE-3.5/issues/79
     Remember to flash to both partitions, `0x10000` and `0x1c0000` !!!

$~$
## Embeded webserver HTML content
* It is packed inside `spiffs.bin` file previously built 


$~$
## Erasing flash memory from the device / factory reset
* `pip install esptool`
* Download boot section `https://github.com/SmartEVSE/SmartEVSE-3/files/8864695/boot_app0.zip`
* extract `boot_app0.zip`
* Replace COM4 port with your SmartEVSE device port (check via Windows device manager)


```
esptool.py.exe --chip esp32 --port COM4 --before default_reset --after hard_reset write_flash 0xe000 boot_app0.bin
```
```
esptool.py.exe --chip esp32 --port COM4 --before default_reset --after hard_reset erase_region 0xe000 0x2000
```
```
esptool.py.exe --chip esp32 --port COM4 erase_flash
```
