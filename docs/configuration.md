
# How to configure your SmartEVSE device
> [!TIP]
 For the first time, it's recommended to setup the device alone without the Sensorbox (NORMAL mode), for the sake of simplicity

> [!NOTE]
> Not all menu options are available at once. Device will display only options applicable to current configuration (ex: solar settings will appear when device is setup in SOLAR mode, etc)

* Set the proper values of the settings belowe (for the first time skip MAINSMET and set MODE to Normal)
* Now you are ready to test/operate your SmartEVSE in its simplest mode, called Normal Mode.
* Plug in your vehicle, if your EV charges at MAX current, and everything works as expected and you are done!
* If you have a MAINSMET (ex: Sensorbox), configure it now. New menu options will appear afterwards
* Now your SmartEVSE is ready for use!

$~$
# All menu options on the LCD screen:
> [!NOTE]
> Per default you are in Normal EVSE mode; you can also choose Smart Mode or Solar Mode, but you will have to configure a MAINSMETer to actually use these modes.


| Menu | Description | Options |
| --- | --- | --- |
| **MODE** | Sets how device will determine maximum charge for EV | **Normal** The EV will charge with the current at MAX *(device maximum capacity)*.<br/><br/>**Smart** The EV will charge with a dynamic charge current, depending on house consumption. It will determine the available charge based on MAINSMET data *(consumption)* and MAINS MAX *(available)*, but it will never exceed device maximum capacity *(MAX)*.<br/><br/>**Solar** The EV will charge when solar power is available
| **CONFIG** | Configure your SmartEVSE with a Type 2 Socket or a fixed cable | **Socket** Your SmartEVSE is connected to a socket, so it will need to sense the cable used for its maximum capacity.<br/><br/>**Fixed** Your SmartEVSE is connected to a fixed cable, so EVSE MAX will determine your maximum charge current.
| **EV MIN** | Set minimum charge current the EV will accept *(Only available in Smart or Solar mode)* | 5 - 16A
| **MAINS MAX** | Set maximum Mains current (contracted power) *(Only available in Smart or Solar mode)* | 10 - 200A
| **EVSE MAX** | Set maximum charge current for the SmartEVSE.<br/>*Note: if CONFIG is set to **Fixed**, set this value according to the maximum current that your fixed cable can carry*. | 10 - 80A
| **LOCK** | Enable or disable the locking actuator *(only available if CONFIG is set to **Socket**)* | **Disabled** No lock is used.<br/><br/>**Solenoid** Dostar, DUOSIDA DSIEC-ELB or Ratio lock.<br/><br/>**Motor** Signal wire reversed, DUOSIDA DSIEC-EL or Phoenix Contact.
| **PWR SHARE** | Formerly known as *LOADBALANCING*. 2 to 8 EVSE’s can be connected via modbus, and their load will be balanced | **Disabled** Single SmartEVSE.<br/><br/>**Master** Set the first SmartEVSE to Master **(make sure there is only one Master).**<br/><br/>**Node1-7** And the other SmartEVSE's to Node 1-7.
| **MAINS METER** | Type of MAINS meter *(only avaiable in Smart or Solar mode)* | **Disabled** No meter connected, only Normal mode possible.<br/><br/>**Sensorbox** Sensorbox will send measurement data to the SmartEVSE.<br/><br/>**API** mains meter data will be fed through the REST API.<br/><br/>**Phoenix C / Finder / Eastron / ...** a Modbus kWh meter is used. *Note that Eastron is for single phase Eastron meters.*
| **GRID** | 1 phase installation or 3 phase installation over 3 or 4 wires *(Only available if MAINS METER is set to **Sensorbox**)* | **3 Wire** 3 phase 3 wires.<br/><br/>**4 Wire** 3 phase 4 wires.<br/><br/>**1 phase** single phase *(note: sensorbox must be connected to L1)*.
| **MAINSADR** | Set the Modbus address for the kWh meter *(Only available if MAINS METER is set to a kWh meter)* | 
| **SOL BOOST** | Exceed MAINS MAX capacity using solar surplus *(Only available in Smart mode and if MAINS METER Sensorbox is in use)*. Useful in 3 phases installations with 1 phase cable and solar panels. | 
| **EV METER** | Set type of EV kWh meter (measures power and charged energy) | **Disabled** No EV meter connected.<br/><br/>**Phoenix C / Finder / Eastron / ...** a Modbus kWh meter is used. *Note that Eastron is for single phase Eastron meters.*
| **CIRCUIT** | Set the max current the EVSE circuit can handle (load balancing). Obeyed in all modes! *(Only available if PWR SHARE is set to Master)* | 10-200A
| **SWITCH** | Set the function of an external switch connected to pin SW. | **Disabled** A push button on io pin SW can be used to STOP charging.<br/><br/>**Access B** A momentary push Button is used to enable/disable access to the charging station.<br/><br/>**Access S** A toggle switch is used to enable/disable access to the charging station.<br/><br/>**Sma-Sol B** A momentary push Button is used to switch between Smart and Solar modes.<br/><br/>**Sma-Sol S** A toggle switch is used to switch between Smart and Solar modes.
| **RCMON** | RCM14-03 Residual Current Monitor is plugged into connector P1 | **Disabled** The RCD option is not used.<br/><br/>**Enabled** When a fault current is detected, the contactor will be opened.
| **RFID** | RFID card reader to enable/disable access to the SmartEVSE *(a maximum of 20 RFID cards can be stored)*. | **Disabled** RFID reader turned off.<br/><br/>**EnableAll** Accept all learned cards for enabling/disabling the SmartEVSE.<br/><br/>**EnableOne** Only allow a single (learned) card to be used for enabling/disabling the SmartEVSE. In this mode the lock (if used) will lock the cable in the charging socket, and the same card is used to unlock it again.<br/><br/>**Learn** Learn a new card and store it into the SmartEVSE. Make sure you stay in the menu when learning cards. Present a card in front of the reader. "Card Stored" will be shown on the LCD.<br/><br/>**Delete** Erase a previous learned card. Hold the card in front of the reader. "Card Deleted" will be shown on the LCD once the card has been deleted.<br/><br/>**DeleteAll** Erase all cards from the SmartEVSE. The cards will be erased once you exit the menu of the SmartEVSE.
| **WIFI** | Enable wifi connection to your LAN. | **Disabled** No wifi connection.<br/><br/>**SetupWifi** The SmartEVSE presents itself as a Wifi Acces Point "smartevse-xxxx"; connect with your phone to that access point, goto http://192.168.4.1/ and configure your Wifi password.<br/><br/>**Enabled** Connect to the previosly setup Wifi.
| **MAX TEMP** | Maximum allowed temperature for your SmartEVSE *(degree celsius)*. | 40 - 65C
| **SOL START** | Current on which the EV should start Solar charging *(only available in Solar MODE)*. | -0  -48A (sum of all phases)
| **SOL STOP** | Stop charging when there is not enough solar power available *(only available in Solar MODE)*. | **Disabled** never stop charging.<br/><br/>**1 - 60** minutes wait.
| **SOL IMPORT** | Allow additional grid power when solar charging *(only available in Solar MODE)*. |  0-20A (sum of all phases).<br/><br/>*NOTE: make sure you enter a value equals or greater than SOL START + EV MIN, otherwise will result in a non-charging SmartEVSE when in Solar mode.*


$~$
# Multiple SmartEVSE controllers on one mains supply (Power Share)
Up to eight SmartEVSE modules can share one mains supply.

## Hardware connections

- Connect the A, B and GND connections from the Master to the Node(s).
  - So A connects to A, B goes to B etc.
  - If you are using Smart/Solar mode, you should connect the A, B , +12V and GND wires from the sensorbox to the same screw terminals of the SmartEVSE! Make sure that the +12V wire from the sensorbox is connected to **only one SmartEVSE**.

## Software configuration
> [!IMPORTANT]
> Make sure there is only one Master, and node numbers are unique!

- Set one SmartEVSE PWR SHARE setting to MASTER, the others to NODE 1-7. 
  - On the Master configure the following:
    - MODE Set this to Smart if a Sensorbox (or configured kWh meter) is used to measure the current draw on the mains supply. It will then dynamically vary the charge current for all connected EV’s. If you are using a dedicated mains supply for the EV’s you can leave this set to Normal.
    - MAINS MAX Set to the maximum current of the MAINS connection. If the sensorbox or other MainsMeter device measures a higher current then this value on one of the phases, it will immediately reduce the current to the EVSE’s
    - CIRCUIT Set this to the maximum current of the EVSE circuit (per phase). This will be split between the connected and charging EV’s.
    - EVSE MAX Set the maximum charging current for the EV connected to -this- SmartEVSE.
    - EV MIN Set to the lowest allowable charging current for all connected EV’s.
  - On the Nodes configure the following:
    - EVSE MAX Set the maximum charging current for the EV connected to -this- SmartEVSE.
