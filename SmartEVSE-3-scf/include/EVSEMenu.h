/*
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
 */

#ifndef __EVSEMENU
#define __EVSEMENU

#include "EVSECluster.h"
#include "EVSEController.h"
#include "EVSELockActuator.h"
#include "EVSEModbus.h"
#include "EVSERFID.h"
#include "EVSERgbLeds.h"
#include "EVSEWifi.h"
#include "i18n.h"

#define MENU_INACTIVITY_TIMEOUT_SECONDS 120

// Node specific configuration
#define MENU_NO_OPTION 0
// Node specific configuration
#define MENU_ENTER 1
#define MENU_CONFIG 2       // Configuration
#define MENU_LOCK 3         // Cable lock
#define MENU_MIN_EV 4       // MIN Charge Current the EV will accept
#define MENU_MAX_CURRENT 5  // MAX Charge Current for this EVSE
#if EVSE_FEATFLAG_ENABLE_POWERSHARE
#define MENU_POWER_SHARE 6  // Load Balance
#endif
#if EVSE_FEATFLAG_ENABLE_EXTSWITCH
#define MENU_SWITCH 7  // External Start/Stop button
#endif
#if EVSE_FEATFLAG_ENABLE_RCMON
#define MENU_RCMON 8  // Residual Current Monitor
#endif
#if EVSE_FEATFLAG_ENABLE_RFID
#define MENU_RFIDREADER 9  // Use RFID reader
#endif
#define MENU_EVMETER 10         // Type of EV electric meter
#define MENU_EVMETERADDRESS 11  // Address of EV electric meter
#define MENU_MODE 12            // EVSE mode
#if EVSE_FEATFLAG_ENABLE_POWERSHARE
#define MENU_CIRCUIT 13  // EVSE Circuit max Current
#endif
#define MENU_GRID 14  // Grid type to which the Sensorbox is connected
#define MENU_SENSORBOX_WIFI 15
#define MENU_SOLAR_BOOST 16
// #define MENU_CALIBRATION 15              // CT calibration value
#define MENU_MAX_MAINS 17                // Max Mains Current
#define MENU_SOLAR_START 18              // Surplus energy start Current
#define MENU_SOLAR_STOP_TIME_MINUTES 19  // Stop solar charging at 6A after this time

#define MENU_IMPORT 20             // Allow grid power when solar charging
#define MENU_MAINSMETER 21         // Type of Mains electric meter
#define MENU_MAINSMETERADDRESS 22  // Address of Mains electric meter
#define MENU_MAINSMETERMEASURE 23  // What does Mains electric meter measure
#if EVSE_FEATFLAG_ENABLE_EVMETER
#define MENU_PVMETER 24         // Type of PV electric meter
#define MENU_PVMETERADDRESS 25  // Address of PV electric meter
#endif
#define MENU_EMCUSTOM_ENDIANESS 26  // Byte order of custom electric meter
#define MENU_EMCUSTOM_DATATYPE 27   // Data type of custom electric meter
#define MENU_EMCUSTOM_FUNCTION 28   // Modbus Function (3/4) of custom electric meter
#define MENU_EMCUSTOM_UREGISTER 29  // Register for Voltage (V) of custom electric meter

#define MENU_EMCUSTOM_UDIVISOR 30   // Divisor for Voltage (V) of custom electric meter (10^x)
#define MENU_EMCUSTOM_IREGISTER 31  // Register for Current (A) of custom electric meter
#define MENU_EMCUSTOM_IDIVISOR 32   // Divisor for Current (A) of custom electric meter (10^x)
#define MENU_EMCUSTOM_PREGISTER 33  // Register for Power (W) of custom electric meter
#define MENU_EMCUSTOM_PDIVISOR 34   // Divisor for Power (W) of custom electric meter (10^x)
#define MENU_EMCUSTOM_EREGISTER 35  // Register for Energy (kWh) of custom electric meter
#define MENU_EMCUSTOM_EDIVISOR 36   // Divisor for Energy (kWh) of custom electric meter (10^x)
#define MENU_EMCUSTOM_READMAX 37    // Maximum register read (not implemented)
#define MENU_MAX_TEMPERATURE 38
#if EVSE_FEATFLAG_ENABLE_LEDS
#define MENU_LEDS 39
#endif

#define MENU_WIFI 40
#define MENU_EXIT 41

// MENU_XXX defines referes to menuEntries array position, can not be empty numbers (gaps)
// due to array iteration over menu elements.
// If position changes in struct, then corresponding DEFINE value must be changed accordly
// Mind EVSEMenu::buildMenuItems() and EVSEModbus::mapModbusRegister2MenuItemID
const struct {
    char LCD[12];
    char Desc[52];
    uint16_t Min;
    uint16_t Max;
    uint16_t Default;
} menuEntries[MENU_EXIT + 1] = {
    //----- Node specific configuration -----
    // Configuration (0:Socket / 1:Fixed Cable)
    {"", I18N_MENU_NOTINMENU, 0, 0, 0},
    {"", I18N_MENU_HOLD2SEC, 0, 0, 0},
    {"CONFIG", I18N_MENU_CONFIG, 0, 1, CONFIG_SOCKET},
    // Cable lock (0:Disable / 1:Solenoid / 2:Motor)
    {"LOCK", I18N_MENU_LOCK, 0, 2, LOCK_DISABLED},
    // Minimal current the EV is happy with (A)
    {"EV MIN", I18N_MENU_MIN, MIN_EV_CURRENT, 16, MIN_EV_CURRENT},
    // Max Charge current (A)
    {"EVSE MAX", I18N_MENU_MAX, 6, (MAX_MAINS_HARD_LIMIT / 10), MAX_DEVICE_CURRENT},
#if EVSE_FEATFLAG_ENABLE_POWERSHARE
    // Load Balance Setting (0:Disable / 1:Master / 2-8:Node)
    {"PWR SHARE", I18N_MENU_POWER_SHARE, 0, CLUSTER_NUM_EVSES, LOAD_BALANCER_DISABLED},
#else
    {"PWR SHARE", "", 0, 0, 0},
#endif
#if EVSE_FEATFLAG_ENABLE_EXTSWITCH
    // External Switch on SW (0:Disable / 1:Access / 2:Smart-Solar)
    {"SWITCH", I18N_MENU_SW, 0, 4, SWITCH_DISABLED},
#else
    {"SWITCH", "", 0, 0, 0},
#endif
// Residual Current Monitor on RCM (0:Disable / 1:Enable)
#if EVSE_FEATFLAG_ENABLE_RCMON
    {"RCMON", I18N_MENU_RCMON, 0, 1, RC_MON_DISABLED},
#else
    {"RCMON", "", 0, 0, 0},
#endif
#if EVSE_FEATFLAG_ENABLE_RFID
    // RFID Reader connected to SW (0:Disable / 1:Enable / 2:Learn / 3:Delete /
    // 4:Delate All)
    {"RFID", I18N_MENU_RFID, 0, 5, RFID_READER_DISABLED},
#else
    {"RFID", "", 0, 0, 0},
#endif

//----- System configuration -----
#if EVSE_FEATFLAG_ENABLE_EVMETER
    // Type of EV electric meter (0: Disabled / Constants EM_*)
    {"EV METER", I18N_MENU_EVEM, 0, MM_CUSTOM, EV_METER_DISABLED},
    // Address of EV electric meter (10 - 247)
    {"EV ADDR", I18N_MENU_EVAD, MIN_EV_METER_ADDRESS, MAX_EV_METER_ADDRESS, EV_METER_ADDRESS},
#else
    {"EV METER", "", 0, 0, 0},
    {"EV ADDR", "", 0, 0, 0},
#endif
    // EVSE mode (0:Normal / 1:Smart)
    {"MODE", I18N_MENU_MODE, 0, 2, MODE_NORMAL},
    // Max current of the EVSE circuit (A)
    {"CIRCUIT", I18N_MENU_CIRCUIT, 10, 160, MAX_CIRCUIT},
    {"GRID", I18N_MENU_GRID, 0, 2, GRID_3WIRE},
    // Sensorbox-2 WiFi Mode (0:Disabled / 1:Enabled / 2:Start Portal)
    {"SB2 WIFI", I18N_MENU_SENSORBOX_WIFI, 0, 2, SB2_WIFI_MODE_DISABLED},
    {"SOL BOOST", I18N_MENU_SOLAR_BOOST, 0, 1, SOLAR_BOOST_DISABLED},
    // Max Mains Amps (hard limit, limited by the MAINS connection) (A)
    // (Mode:Smart/Solar)
    {"MAINS MAX", I18N_MENU_MAINS, 10, 200, MAX_MAINS},
    // Start Surplus Current (A) in Solar mode
    {"SOL START", I18N_MENU_START, 0, 48, SOLAR_START_CURRENT},
    // Stop time (min)
    {"SOL STOP", I18N_MENU_STOP, 0, 60, SOLAR_STOP_TIME_MINUTES},

    // Import Current from Grid (A) in Solar mode
    {"SOL IMPORT", I18N_MENU_IMPORT, 0, 20, SOLAR_IMPORT_CURRENT},
    // Type of Mains electric meter (0: Disabled / Constants EM_*)
    {"MAINS METER", I18N_MENU_MAINEM, 0, MM_CUSTOM, MM_SENSORBOX},
    // Address of Mains electric meter (5 - 254)
    {"MAINS ADDR", I18N_MENU_MAINAD, MIN_EV_METER_ADDRESS, MAX_EV_METER_ADDRESS, MAINS_METER_ADDRESS},
    // What does Mains electric meter measure (0: Mains (Home+EVSE+PV) / 1:
    // Home+EVSE / 2: Home)
    {"MAINS MSRE", I18N_MENU_MAINM, 0, 1, MAINS_METER_MEASURE},
#if EVSE_FEATFLAG_ENABLE_EVMETER
    // Type of PV electric meter (0: Disabled / Constants EM_*)
    {"PV METER", I18N_MENU_PVEM, 0, MM_CUSTOM, PV_METER_DISABLED},
    // Address of PV electric meter (5 - 254)
    {"PV ADDR", I18N_MENU_PVAD, MIN_EV_METER_ADDRESS, MAX_EV_METER_ADDRESS, PV_METER_ADDRESS},
#else
    {"PV METER", "", 0, 0, 0},
    {"PV ADDR", "", 0, 0, 0},
#endif
    // Byte order of custom electric meter
    {"BYTE ORD", I18N_MENU_EMBO, 0, 3, EMCUSTOM_ENDIANESS},
    // Data type of custom electric meter
    {"DATATYPE", I18N_MENU_EMDATA, 0, MB_DATATYPE_MAX - 1, EMCUSTOM_DATATYPE},
    // Modbus Function of custom electric meter
    {"FUNCTION", I18N_MENU_EMFUNC, 3, 4, EMCUSTOM_FUNCTION},
    // Starting register for voltage of custom electric meter
    {"VOL REGI", I18N_MENU_EMUREG, 0, 65530, EMCUSTOM_UREGISTER},

    // Divisor for voltage of custom electric meter
    {"VOL DIVI", I18N_MENU_EMUDIV, 0, 7, EMCUSTOM_UDIVISOR},
    // Starting register for current of custom electric meter
    {"CUR REGI", I18N_MENU_EMIREG, 0, 65530, EMCUSTOM_IREGISTER},
    // Divisor for current of custom electric meter
    {"CUR DIVI", I18N_MENU_EMIDIV, 0, 7, EMCUSTOM_IDIVISOR},
    // Starting register for power of custom electric meter
    {"POW REGI", I18N_MENU_EMPREG, 0, 65534, EMCUSTOM_PREGISTER},
    // Divisor for power of custom electric meter
    {"POW DIVI", I18N_MENU_EMPDIV, 0, 7, EMCUSTOM_PDIVISOR},
    // Starting register for energy of custom electric meter
    {"ENE REGI", I18N_MENU_EMEREG, 0, 65534, EMCUSTOM_EREGISTER},
    // Divisor for energy of custom electric meter
    {"ENE DIVI", I18N_MENU_EMEDIV, 0, 7, EMCUSTOM_EDIVISOR},
    {"READ MAX", I18N_MENU_EMREAD, 3, 255, 3},
    {"MAX TEMP", I18N_MENU_TEMPERATURE, 40, 65, DEFAULT_MAX_TEMPERATURE},
#if EVSE_FEATFLAG_ENABLE_LEDS
    {"LEDS", I18N_MENU_LEDS, 0, 1, DEFAULT_LEDS_ENABLED_VALUE},
#else
    {"LEDS", "", 0, 0, 0},
#endif

    {"WIFI", I18N_MENU_WIFI, 0, 2, WIFI_MODE_DISABLED},
    {"EXIT", I18N_MENU_EXIT, 0, 0, 0}};

class EVSEMenu {
   public:
    EVSEMenu() {};
    void setup();
    uint16_t getMenuItemValue(uint8_t nav);
    uint8_t setMenuItemValue(uint8_t nav, uint16_t val);
    void onButtonChanged(uint8_t Buttons);
    bool shouldRedrawMenu();
    uint8_t getPosInMenu();
    uint8_t getMenuItemsCount() { return menuItemsCount; };
    const char* getMenuItemi18nText(uint8_t nav);
    void resetInactivityTimer();
    void updateSettings();

    uint8_t MenuItems[MENU_EXIT];
    uint8_t currentMenuOption = MENU_NO_OPTION;
    uint8_t subMenu = 0;

   protected:
    TaskHandle_t inactivityTaskHandle = NULL;
    static void onInactivityTaskHandle(EVSEMenu* myself);
    void beginInactivityTask();
    void endInactivityTask();
    void checkExitMenuOnInactivity();

   private:
    void handleButtons(uint8_t buttons);
    void handleButtonsReleased();
    void handleButtonOPressed();
    void handleButtonBothArrowsPressed();
    void handleButtonArrowPressed(uint8_t buttons);
    void buildMenuItems();
    uint16_t circleValues(uint8_t buttons, uint16_t value, uint16_t minValue, uint16_t maxValue);
    void prevNextMenuEntry(uint8_t buttons);

    unsigned long buttonTimer = 0;
    uint8_t buttonReleased = 0;
    uint16_t buttonRepeat = 0;
    uint8_t menuItemsCount = 0;
    unsigned long inactivityTimer = 0;
};

extern EVSEMenu evseMenu;

#endif
