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

#include "EVSEMenu.h"

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>

#include "EVSEButtons.h"
#include "EVSECluster.h"
#include "EVSEController.h"
#include "EVSELockActuator.h"
#include "EVSELogger.h"
#include "EVSEModbus.h"
#include "EVSERFID.h"
#include "EVSERgbLeds.h"
#include "EVSEScreen.h"
#include "EVSEWifi.h"
#include "utils.h"

void EVSEMenu::buildMenuItems() {
    // MENU_XXX defines referes to menuEntries array position, can not be empty numbers (gaps)
    // due to array iteration over menu elements.
    // If position changes in struct, then corresponding DEFINE value must be changed accordly
    // Mind EVSEMenu::buildMenuItems() and EVSEModbus::mapModbusRegister2MenuItemID

    uint8_t m = 0;

    MenuItems[m++] = MENU_CONFIG;
    if (evseController.config == CONFIG_SOCKET) {
        MenuItems[m++] = MENU_LOCK;
    }

    MenuItems[m++] = MENU_MODE;

    // Mode Smart/Solar and Load Balancing Disabled/Master
    if (evseController.mode != MODE_NORMAL && evseCluster.amIMasterOrLBDisabled()) {
        MenuItems[m++] = MENU_MIN_EV;
        MenuItems[m++] = MENU_MAX_MAINS;
    }

    MenuItems[m++] = MENU_MAX_CURRENT;

    // Mode Smart/Solar and Load Balancing Disabled/Master
    if (evseController.mode == MODE_SOLAR && evseCluster.amIMasterOrLBDisabled()) {
        MenuItems[m++] = MENU_SOLAR_START;
        MenuItems[m++] = MENU_SOLAR_STOP_TIME_MINUTES;
        MenuItems[m++] = MENU_IMPORT;
    }

    // Mode Smart/Solar
    if (evseController.mode != MODE_NORMAL) {
        if (evseCluster.amIMasterOrLBDisabled()) {
            MenuItems[m++] = MENU_MAINSMETER;

            // Sensorbox
            if (evseModbus.mainsMeter == MM_SENSORBOX) {
                MenuItems[m++] = MENU_GRID;
                if (evseModbus.SB2.SoftwareVer == 0x01) {
                    MenuItems[m++] = MENU_SENSORBOX_WIFI;
                }

                if (evseController.mode == MODE_SMART) {
                    MenuItems[m++] = MENU_SOLAR_BOOST;
                }
            } else if (evseModbus.mainsMeter != MAINS_METER_DISABLED) {
                MenuItems[m++] = MENU_MAINSMETERADDRESS;
                MenuItems[m++] = MENU_MAINSMETERMEASURE;
                MenuItems[m++] = MENU_SOLAR_BOOST;

                // PV not measured by Mains electric meter?
                if (evseModbus.mainsMeterMeasure) {
                    MenuItems[m++] = MENU_PVMETER;
                    if (evseModbus.pvMeter != PV_METER_DISABLED) {
                        MenuItems[m++] = MENU_PVMETERADDRESS;
                    }
                }
            }
        }

        MenuItems[m++] = MENU_EVMETER;
        if (evseModbus.evMeter != EV_METER_DISABLED) {
            MenuItems[m++] = MENU_EVMETERADDRESS;
        }

        if (evseCluster.amIMasterOrLBDisabled()) {
            // Custom electric meter used?
            if (evseModbus.mainsMeter == MM_CUSTOM || evseModbus.pvMeter == MM_CUSTOM ||
                evseModbus.evMeter == MM_CUSTOM) {
                MenuItems[m++] = MENU_EMCUSTOM_ENDIANESS;
                MenuItems[m++] = MENU_EMCUSTOM_DATATYPE;
                MenuItems[m++] = MENU_EMCUSTOM_FUNCTION;
                MenuItems[m++] = MENU_EMCUSTOM_UREGISTER;
                MenuItems[m++] = MENU_EMCUSTOM_UDIVISOR;
                MenuItems[m++] = MENU_EMCUSTOM_IREGISTER;
                MenuItems[m++] = MENU_EMCUSTOM_IDIVISOR;
                MenuItems[m++] = MENU_EMCUSTOM_PREGISTER;
                MenuItems[m++] = MENU_EMCUSTOM_PDIVISOR;
                MenuItems[m++] = MENU_EMCUSTOM_EREGISTER;
                MenuItems[m++] = MENU_EMCUSTOM_EDIVISOR;
            }
        }
    }

    MenuItems[m++] = MENU_POWER_SHARE;
    if (evseCluster.isLoadBalancerMaster()) {
        MenuItems[m++] = MENU_CIRCUIT;
    }

    MenuItems[m++] = MENU_SWITCH;
    MenuItems[m++] = MENU_RCMON;
    MenuItems[m++] = MENU_RFIDREADER;

    MenuItems[m++] = MENU_MAX_TEMPERATURE;
    MenuItems[m++] = MENU_LEDS;
    MenuItems[m++] = MENU_WIFI;
    MenuItems[m++] = MENU_EXIT;

    menuItemsCount = m;
}

uint16_t EVSEMenu::getMenuItemValue(uint8_t nav) {
    switch (nav) {
        case MENU_CONFIG:
            return evseController.config;
        case MENU_MODE:
        case NODE_STATUS_MODE:
            return evseController.mode;
        case MENU_SOLAR_START:
            return evseController.solarStartCurrent;
        case MENU_SOLAR_STOP_TIME_MINUTES:
            return evseController.solarStopTimeMinutes;
        case MENU_IMPORT:
            return evseController.solarImportCurrent;
        case MENU_POWER_SHARE:
            return evseCluster.getLoadBl();
        case MENU_MAX_MAINS:
            return evseController.maxMains;
        case MENU_MIN_EV:
            return evseController.minEVCurrent;
        case MENU_MAX_CURRENT:
            return evseController.maxDeviceCurrent;
        case MENU_CIRCUIT:
            return evseCluster.getMaxCircuit();
        case MENU_LOCK:
            return evseLockActuator.getLockType();
        case MENU_SWITCH:
            return evseController.externalSwitch;
        case MENU_RCMON:
            return evseController.RCmon;
        case MENU_GRID:
            return evseModbus.grid;
        case MENU_MAINSMETER:
            return evseModbus.mainsMeter;
        case MENU_MAINSMETERADDRESS:
            return evseModbus.mainsMeterAddress;
        case MENU_MAINSMETERMEASURE:
            return evseModbus.mainsMeterMeasure;
        case MENU_PVMETER:
            return evseModbus.pvMeter;
        case MENU_PVMETERADDRESS:
            return evseModbus.pvMeterAddress;
        case MENU_EVMETER:
            return evseModbus.evMeter;
        case MENU_EVMETERADDRESS:
            return evseModbus.evMeterAddress;
        case MENU_EMCUSTOM_ENDIANESS:
            return evseModbus.EMConfig[MM_CUSTOM].Endianness;
        case MENU_EMCUSTOM_DATATYPE:
            return evseModbus.EMConfig[MM_CUSTOM].DataType;
        case MENU_EMCUSTOM_FUNCTION:
            return evseModbus.EMConfig[MM_CUSTOM].Function;
        case MENU_EMCUSTOM_UREGISTER:
            return evseModbus.EMConfig[MM_CUSTOM].URegister;
        case MENU_EMCUSTOM_UDIVISOR:
            return evseModbus.EMConfig[MM_CUSTOM].UDivisor;
        case MENU_EMCUSTOM_IREGISTER:
            return evseModbus.EMConfig[MM_CUSTOM].IRegister;
        case MENU_EMCUSTOM_IDIVISOR:
            return evseModbus.EMConfig[MM_CUSTOM].IDivisor;
        case MENU_EMCUSTOM_PREGISTER:
            return evseModbus.EMConfig[MM_CUSTOM].PRegister;
        case MENU_EMCUSTOM_PDIVISOR:
            return evseModbus.EMConfig[MM_CUSTOM].PDivisor;
        case MENU_EMCUSTOM_EREGISTER:
            return evseModbus.EMConfig[MM_CUSTOM].ERegister;
        case MENU_EMCUSTOM_EDIVISOR:
            return evseModbus.EMConfig[MM_CUSTOM].EDivisor;
        case MENU_RFIDREADER:
            return evseRFID.RFIDReader;
        case MENU_WIFI:
            return evseWifi.getWifiMode();
        case MENU_MAX_TEMPERATURE:
            return evseController.maxTemperature;
        case MENU_LEDS:
            return evseRgbLeds.ledsEnabled ? 1 : 0;
        case MENU_SOLAR_BOOST:
            return evseController.isSolarBoost() ? 1 : 0;
        case MENU_SENSORBOX_WIFI:
            return evseModbus.getSensorboxWifiMode();
        // Status writeable
        case NODE_STATUS_STATE:
            return evseController.state;
        case NODE_STATUS_ERROR:
            return evseController.errorFlags;
        case NODE_STATUS_CHARGECURRENT:
            return evseController.getChargeCurrent();
        case NODE_STATUS_SOLAR_TIMER:
            return evseController.solarStopTimer;
        case NODE_STATUS_ACCESSBIT:
            return evseRFID.rfidAccessBit;
        case NODE_STATUS_CONFIG_CHANGED:
            return evseController.statusConfigChanged;
        // Status readonly
        case NODE_STATUS_CABLEMAX:
            return evseController.getCableMaxCapacity();
        case NODE_STATUS_TEMP:
            return (signed int)evseController.temperature + 273;

            // case STATUS_SERIAL:
            //   return serialnr;

        default:
            return 0;
    }
}

uint8_t EVSEMenu::setMenuItemValue(uint8_t nav, uint16_t val) {
    // Check value between limits
    if (nav < MENU_EXIT) {
        if (val < menuEntries[nav].Min || val > menuEntries[nav].Max) {
            // Invalid value
            return 0;
        }
    }

    switch (nav) {
        case MENU_CONFIG:
            evseController.config = val;
            break;
        case NODE_STATUS_MODE:
            // Do not change Charge Mode when set to Normal or Load Balancing is
            // disabled
            if (evseController.mode == MODE_NORMAL || evseCluster.isLoadBalancerDisabled()) {
                break;
            }
        case MENU_MODE:
            evseController.switchMode(val);
            break;
        case MENU_SOLAR_START:
            evseController.solarStartCurrent = val;
            break;
        case MENU_SOLAR_STOP_TIME_MINUTES:
            evseController.solarStopTimeMinutes = val;
            break;
        case MENU_IMPORT:
            evseController.solarImportCurrent = val;
            break;
        case MENU_POWER_SHARE:
            evseModbus.configureModbusMode(val);
            evseCluster.setLoadBl(val);
            break;
        case MENU_MAX_MAINS:
            evseController.maxMains = val;
            break;
        case MENU_MIN_EV:
            evseController.minEVCurrent = val;
            break;
        case MENU_MAX_CURRENT:
            evseController.maxDeviceCurrent = val;
            break;
        case MENU_CIRCUIT:
            evseCluster.setMaxCircuit(val);
            break;
        case MENU_LOCK:
            evseLockActuator.setLockType(val);
            break;
        case MENU_SWITCH:
            evseController.externalSwitch = val;
            break;
        case MENU_RCMON:
            evseController.RCmon = val;
            break;
        case MENU_GRID:
            evseModbus.grid = val;
            break;
        case MENU_MAINSMETER:
            evseModbus.mainsMeter = val;
            break;
        case MENU_MAINSMETERADDRESS:
            evseModbus.mainsMeterAddress = val;
            break;
        case MENU_MAINSMETERMEASURE:
            evseModbus.mainsMeterMeasure = val;
            break;
        case MENU_PVMETER:
            evseModbus.setPvMeter(val);
            break;
        case MENU_PVMETERADDRESS:
            evseModbus.pvMeterAddress = val;
            break;
        case MENU_EVMETER:
            evseModbus.evMeter = val;
            break;
        case MENU_EVMETERADDRESS:
            evseModbus.evMeterAddress = val;
            break;
        case MENU_EMCUSTOM_ENDIANESS:
            evseModbus.EMConfig[MM_CUSTOM].Endianness = val;
            break;
        case MENU_EMCUSTOM_DATATYPE:
            evseModbus.EMConfig[MM_CUSTOM].DataType = (mb_datatype)val;
            break;
        case MENU_EMCUSTOM_FUNCTION:
            evseModbus.EMConfig[MM_CUSTOM].Function = val;
            break;
        case MENU_EMCUSTOM_UREGISTER:
            evseModbus.EMConfig[MM_CUSTOM].URegister = val;
            break;
        case MENU_EMCUSTOM_UDIVISOR:
            evseModbus.EMConfig[MM_CUSTOM].UDivisor = val;
            break;
        case MENU_EMCUSTOM_IREGISTER:
            evseModbus.EMConfig[MM_CUSTOM].IRegister = val;
            break;
        case MENU_EMCUSTOM_IDIVISOR:
            evseModbus.EMConfig[MM_CUSTOM].IDivisor = val;
            break;
        case MENU_EMCUSTOM_PREGISTER:
            evseModbus.EMConfig[MM_CUSTOM].PRegister = val;
            break;
        case MENU_EMCUSTOM_PDIVISOR:
            evseModbus.EMConfig[MM_CUSTOM].PDivisor = val;
            break;
        case MENU_EMCUSTOM_EREGISTER:
            evseModbus.EMConfig[MM_CUSTOM].ERegister = val;
            break;
        case MENU_EMCUSTOM_EDIVISOR:
            evseModbus.EMConfig[MM_CUSTOM].EDivisor = val;
            break;
        case MENU_RFIDREADER:
            evseRFID.setRFIDReader(val);
            break;
        case MENU_WIFI:
            evseWifi.setWifiMode(val);
            break;
        case MENU_MAX_TEMPERATURE:
            evseController.maxTemperature = val;
            break;
        case MENU_LEDS:
            evseRgbLeds.ledsEnabled = (val == 1);
            break;
        case MENU_SOLAR_BOOST:
            evseController.setSolarBoost(val == 1);
            break;
        // Status writeable
        case NODE_STATUS_STATE:
            evseController.setState(val);
            break;

        case NODE_STATUS_ERROR:
            evseController.onNodeReceivedError(val);
            break;

        case NODE_STATUS_CHARGECURRENT:
            evseCluster.setOverrideCurrent(val);
            break;

        case NODE_STATUS_SOLAR_TIMER:
            evseController.setSolarStopTimer(val);
            break;

        case NODE_STATUS_ACCESSBIT:
            if (val == 0 || val == 1) {
                evseController.setAccess(val);
            }
            break;

        case NODE_STATUS_CONFIG_CHANGED:
            evseController.statusConfigChanged = val;
            break;

        case MENU_SENSORBOX_WIFI:
            evseModbus.setSensorboxWifiMode(val);
            break;

        default:
            return 0;
    }

    return 1;
}

const char* EVSEMenu::getMenuItemi18nText(uint8_t nav) {
    // static to avoid "warning: address of local variable 'Str' returned"
    static char Str[12];
    unsigned int value = getMenuItemValue(nav);

    switch (nav) {
        case MENU_CONFIG:
            return (evseController.config == CONFIG_FIXED_CABLE) ? i18nStrFixed : i18nStrSocket;

        case MENU_MODE:
            if (evseController.mode == MODE_SMART)
                return i18nStrSmart;
            if (evseController.mode == MODE_SOLAR)
                return i18nStrSolar;

            return i18nStrNormal;

        case MENU_SOLAR_START:
            sprintf(Str, I18N_MENUSTART_FORMAT, value);
            return Str;

        case MENU_SOLAR_STOP_TIME_MINUTES:
            if (value) {
                sprintf(Str, I18N_MENUSTOP_FORMAT, value);
                return Str;
            } else
                return i18nStrDisabled;
        case MENU_POWER_SHARE:
            /*if (controller->externalMaster && value == 1)
                return "Node 0";
            else*/
            return i18nStrLoadBl[evseCluster.getLoadBl()];
        case MENU_MAX_MAINS:
        case MENU_MIN_EV:
        case MENU_MAX_CURRENT:
        case MENU_CIRCUIT:
        case MENU_IMPORT:
            sprintf(Str, I18N_MENUIMPORT_FORMAT, value);
            return Str;
        case MENU_LOCK:
            switch (evseLockActuator.getLockType()) {
                case LOCK_SOLENOID:
                    return i18nStrSolenoid;
                case LOCK_MOTOR:
                    return i18nStrMotor;

                default:
                    return i18nStrDisabled;
            }
        case MENU_SWITCH:
            return i18nStrSwitch[evseController.externalSwitch];
        case MENU_RCMON:
            return (evseController.RCmon == RC_MON_DISABLED) ? i18nStrDisabled : i18nStrEnabled;
        case MENU_MAINSMETER:
        case MENU_PVMETER:
        case MENU_EVMETER:
            return geti18nStrMeterText(value);
        case MENU_GRID:
            return i18nStrGrid[evseModbus.grid];
        case MENU_SENSORBOX_WIFI:
            return i18nStrSensorboxWifi[evseModbus.getSensorboxWifiMode()];
        case MENU_MAINSMETERADDRESS:
        case MENU_PVMETERADDRESS:
        case MENU_EVMETERADDRESS:
        case MENU_EMCUSTOM_UREGISTER:
        case MENU_EMCUSTOM_IREGISTER:
        case MENU_EMCUSTOM_PREGISTER:
        case MENU_EMCUSTOM_EREGISTER:
            if (value < 0x1000) {
                // This just fits on the LCD.
                sprintf(Str, "%u (%02X)", value, value);
            } else {
                sprintf(Str, "%u %X", value, value);
            }
            return Str;

        case MENU_MAINSMETERMEASURE:
            return (evseModbus.mainsMeterMeasure == MAINS_METER_ADDRESS) ? i18nStrMainsAll : i18nStrMainsHomeEVSE;

        case MENU_EMCUSTOM_ENDIANESS:
            switch (value) {
                case 0:
                    return "LBF & LWF";
                case 1:
                    return "LBF & HWF";
                case 2:
                    return "HBF & LWF";
                case 3:
                    return "HBF & HWF";
                default:
                    return "";
            }

        case MENU_EMCUSTOM_DATATYPE:
            switch (value) {
                case MB_DATATYPE_INT16:
                    return "INT16";
                case MB_DATATYPE_INT32:
                    return "INT32";
                case MB_DATATYPE_FLOAT32:
                    return "FLOAT32";
            }
        case MENU_EMCUSTOM_FUNCTION:
            switch (value) {
                case 3:
                    return i18nStrEmCustomFunction3;
                case 4:
                    return i18nStrEmCustomFunction4;
                default:
                    return "";
            }
        case MENU_EMCUSTOM_UDIVISOR:
        case MENU_EMCUSTOM_IDIVISOR:
        case MENU_EMCUSTOM_PDIVISOR:
        case MENU_EMCUSTOM_EDIVISOR:
            sprintf(Str, "%lu", pow_10[value]);
            return Str;
        case MENU_RFIDREADER:
            return i18nStrRFIDReader[evseRFID.RFIDReader];
        case MENU_WIFI:
            return i18nStrWiFi[evseWifi.getWifiMode()];
        case MENU_MAX_TEMPERATURE:
            sprintf(Str, I18N_TEMPERATURE_FORMAT, value, 0x0C);
            return Str;
        case MENU_LEDS:
            return i18nStrLeds[evseRgbLeds.ledsEnabled ? 1 : 0];
        case MENU_SOLAR_BOOST:
            return i18nStrSolarBoost[evseController.isSolarBoost() ? 1 : 0];
        case MENU_EXIT:
            return i18nStrExitMenu;
        default:
            return "";
    }
}

uint8_t EVSEMenu::getPosInMenu() {
    for (uint8_t i = 0; i < menuItemsCount; i++) {
        if (MenuItems[i] == currentMenuOption)
            return i + 1u;
    }

    return 1;
}

/**
 * Navigate left/right in a menu of int options
 */
uint16_t EVSEMenu::circleValues(uint8_t buttons, uint16_t value, uint16_t minValue, uint16_t maxValue) {
    switch (buttons) {
        case BUTTON_RIGHT_MASK:
            return (value >= maxValue) ? minValue : value + 1;

        case BUTTON_LEFT_MASK:
            return (value <= minValue) ? maxValue : value - 1;

        default:
            return value;
    }
}

/**
 * Navigate left/right in a menu of char array options
 */
void EVSEMenu::prevNextMenuEntry(uint8_t buttons) {
    // Rebuild menu, maybe some user selection added/removed menu entries
    buildMenuItems();

    uint8_t posInMenu = getPosInMenu();
    posInMenu = circleValues(buttons, --posInMenu, 0, menuItemsCount - 1u);
    currentMenuOption = MenuItems[posInMenu];
}

void EVSEMenu::handleButtonsReleased() {
    // Button 2 released before entering menu
    if (currentMenuOption == MENU_ENTER) {
        currentMenuOption = MENU_NO_OPTION;
    }

    // EVSELogger::debug("[EVSEMenu] handleButtonsReleased");
    buttonReleased = 0;
    buttonRepeat = 0;
    // debounce keys (blocking)
    delay(10);
}

void EVSEMenu::handleButtonOPressed() {
    switch (currentMenuOption) {
        case MENU_NO_OPTION:
            // Button 2 just pressed, must hold for 2 seconds to enter menu
            if (buttonReleased == 0) {
                currentMenuOption = MENU_ENTER;
                buttonTimer = millis() + 2000;
            }
            break;

        case MENU_ENTER:
            // Button o was pressed for 2 seconds, then we enter the menu
            if (millis() >= buttonTimer) {
                currentMenuOption = MENU_CONFIG;
                buttonReleased = 1;
            }
            break;

        default:
            if (buttonReleased != 0) {
                return;
            }

            buttonReleased = 1;
            // In submenu => exit submenu
            if (subMenu) {
                subMenu = 0;
                return;
            }

            // Entering a submenu
            subMenu = 1;
            // Exit Main Menu
            if (currentMenuOption == MENU_EXIT) {
                EVSELogger::info("[EVSEMenu] Exit pressed. Saving menu options...");
                currentMenuOption = MENU_NO_OPTION;
                subMenu = 0;
                // Skip updating of the LCD
                buttonReleased = 2;
                updateSettings();
                delay(100);

                endInactivityTask();
            }
    }
}

void EVSEMenu::updateSettings() {
    evseController.updateSettings();
    evseWifi.updateSettings();
    evseRFID.updateSettings();
    evseLockActuator.updateSettings();
    evseModbus.updateSettings();
    evseCluster.updateSettings();
    evseRgbLeds.updateSettings();

    evseController.statusConfigChanged = 1;
}

void EVSEMenu::handleButtonBothArrowsPressed() {
    if (buttonReleased != 0) {
        return;
    }

    buttonReleased = 1;

    // re-initialize LCD
    evseScreen.resetLCD();
}

void EVSEMenu::handleButtonArrowPressed(uint8_t buttons) {
    switch (buttonReleased) {
        case 0:
        case 1:
            buttonReleased = 1;

            // We are circling between submenu options
            if (subMenu) {
                uint16_t value = 0;

                switch (currentMenuOption) {
                    case MENU_EVMETER:
                        // do not display the Sensorbox here
                        value = getMenuItemValue(currentMenuOption);
                        do {
                            value = circleValues(buttons, value, menuEntries[currentMenuOption].Min,
                                                 menuEntries[currentMenuOption].Max);
                        } while (value == MM_SENSORBOX);
                        setMenuItemValue(currentMenuOption, value);
                        break;

                    default:
                        value = getMenuItemValue(currentMenuOption);
                        value = circleValues(buttons, value, menuEntries[currentMenuOption].Min,
                                             menuEntries[currentMenuOption].Max);
                        setMenuItemValue(currentMenuOption, value);
                        break;
                }

                return;
            }

            // Move to next / prev menu entry
            prevNextMenuEntry(buttons);
            break;

            /*  case 2:
                // Repeat button after 0.5 second
                if (buttonRepeat == 0)
                {
                  buttonRepeat = 500;
                  buttonTimer = millis() + buttonRepeat;
                }
                break;*/

        default:
            // Repeat button after 0.5 second
            if (buttonReleased == 2 && buttonRepeat == 0) {
                buttonRepeat = 500;
                buttonTimer = millis() + buttonRepeat;
            }

            // Repeat button if buttonTimer has passed
            if (buttonRepeat && (millis() >= buttonTimer)) {
                buttonReleased = 0;
                if (buttonRepeat > 1) {
                    buttonRepeat -= (buttonRepeat / 8);
                    buttonTimer = millis() + buttonRepeat;
                }
            }
    }
}

void EVSEMenu::handleButtons(uint8_t buttons) {
    switch (buttons) {
        case BUTTON_NONE_MASK:
            handleButtonsReleased();
            break;

        case BUTTON_O_MASK:
            handleButtonOPressed();
            break;

        case BUTTON_LEFT_AND_RIGHT_MASK:
            // Buttons < and > pressed in order to reset
            handleButtonBothArrowsPressed();
            break;

        case BUTTON_LEFT_MASK:
        case BUTTON_RIGHT_MASK:
            handleButtonArrowPressed(buttons);
            break;
    }
}

void EVSEMenu::resetInactivityTimer() {
    inactivityTimer = millis();
}

void EVSEMenu::onInactivityTaskHandle(EVSEMenu* myself) {
    myself->checkExitMenuOnInactivity();
}

void EVSEMenu::beginInactivityTask() {
    if (inactivityTaskHandle == NULL) {
        EVSELogger::debug("[EVSEMenu] Starting inactivity task");
        // Thread to exit menu after 120 seconds of inactivity
        xTaskCreate((TaskFunction_t)&onInactivityTaskHandle, "onInactivity", 16384, &evseMenu, 0,
                    &inactivityTaskHandle);
    }
}

void EVSEMenu::endInactivityTask() {
    currentMenuOption = MENU_NO_OPTION;
}

void EVSEMenu::checkExitMenuOnInactivity() {
    // Exit Setup menu after 20 seconds, unless rfid submenu so it will not exit
    // the menu when learning/deleting cards
    while (currentMenuOption != MENU_NO_OPTION) {
        if ((millis() - inactivityTimer) > (MENU_INACTIVITY_TIMEOUT_SECONDS * 1000)) {
            // Corner case: RFID reader is storaging or deleting data... it might take
            // longer than timeout
            if (currentMenuOption == MENU_RFIDREADER && subMenu) {
                EVSELogger::debug("EVSEMenu] Canceling menu exit due to RFID submenu");
                continue;
            }

            // Corner case: WiFi portal and WiFi configuration
            if ((currentMenuOption == MENU_WIFI || currentMenuOption == MENU_SENSORBOX_WIFI) && subMenu) {
                EVSELogger::debug("EVSEMenu] Canceling menu exit due to WIFI submenu");
                continue;
            }

            EVSELogger::debug("[EVSEMenu] Exiting menu on timeout...");
            currentMenuOption = MENU_NO_OPTION;
            // don't save, but restore settings
            evseController.resetSettings();
            delay(100);
            evseWifi.resetSettings();
            delay(100);
            evseModbus.resetSettings();
            delay(100);
            evseCluster.resetSettings();
            delay(100);
            evseRFID.resetSettings();
            delay(100);
            evseLockActuator.resetSettings();
            delay(100);
            break;
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    EVSELogger::debug("[EVSEMenu] Ending inactivity task...");
    inactivityTaskHandle = NULL;
    vTaskDelete(NULL);
    EVSELogger::debug("[EVSEMenu] Task DELETED!");
}

/**
 * Called when one of the SmartEVSE buttons is pressed
 *
 * @param buttons: < o >
 *          Value: 1 2 4
 *            Bit: 0:Pressed / 1:Released
 */
void EVSEMenu::onButtonChanged(uint8_t buttons) {
    resetInactivityTimer();
    handleButtons(buttons);

    if (buttonReleased == 1 || currentMenuOption == MENU_ENTER) {
        // Set value to 2, so that LCD will be updated only once
        buttonReleased = 2;
    }

    beginInactivityTask();
}

bool EVSEMenu::shouldRedrawMenu() {
    return (buttonReleased == 1 || currentMenuOption == MENU_ENTER ||
            (currentMenuOption == MENU_RFIDREADER && subMenu) || (currentMenuOption == MENU_WIFI && subMenu) ||
            (currentMenuOption == MENU_SENSORBOX_WIFI && subMenu));
}

void EVSEMenu::setup() {
    buildMenuItems();
};

EVSEMenu evseMenu;