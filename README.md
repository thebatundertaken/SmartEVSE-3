SmartEVSE v3
=========
[![GitHub release](https://img.shields.io/github/release/thebatundertaken/SmartEVSE-3.svg)](https://github.com/thebatundertaken/SmartEVSE-3/releases)

Changes to original firmware v3.0.0
- Adaptation for a 3 phases installation with 1 phase fixed cable
- New solar boost feature to allow charging over max mains when solar power surplus available, useful for 3 phases installation and 1 phase charging cable
- Fixed smart mode power rebalance on 3 phases grid
- Device screen on Smart/Solar mode renders decimal amps for phases consumption
- Added scheduled charging with switch on and off time (operating hours)
- New web status page (UI design, Rest API and no webSockets)
- New max EVSE temperature menu and settings
- New RGB leds enabled/disabled menu and settings
- Massive code refactor to C++ objects
- Improved timers and tasks
- Special thanks to **serkri** for his work (https://github.com/serkri/SmartEVSE-3)

![Status page](./pictures/statuspage.jpg)

![Status page](./pictures/statuspage-mobile.jpg)

![Status page](./pictures/power-monitor.jpg)

![Status page](./pictures/device.jpg)

$~$
## Device configuration

[Device configuration and all menu options explained](docs/configuration.md)<br>
[Official SmartEVSE 3 manual](manual/SmartEVSEv3_install_v3.0.pdf)<br>

## Setting up WiFi

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
## Building the firmware
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
### Option 2: USB flashing (Windows OS)
* InstallDriver for Virtual Port https://www.silabs.com/documents/public/software/CP210x_VCP_Windows.zip
* Open `platformio.ini` file and replace COM4 port with your SmartEVSE device port (check via Windows device manager)
* Upload via USB configured in platformio.ini: platformio run --target upload
* Flash `spiffs.bin` archive (recommended via WiFi flashing)

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


$~$
# Official documentation

[Hardware installation](https://github.com/SmartEVSE/SmartEVSE-3/blob/master/docs/installation.md)<br>
[Operation](https://github.com/SmartEVSE/SmartEVSE-3/blob/master/docs/operation.md)<br>
[Building and Flashing the firmware](https://github.com/SmartEVSE/SmartEVSE-3/blob/master/docs/building_flashing.md)<br>
