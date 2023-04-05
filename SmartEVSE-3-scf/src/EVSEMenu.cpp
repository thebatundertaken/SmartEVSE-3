#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>

#include "EVSEButtons.h"
#include "EVSEController.h"
#include "EVSELockActuator.h"
#include "EVSEMenu.h"
#include "EVSEModbus.h"
#include "EVSERFID.h"
#include "EVSERgbLeds.h"
#include "EVSEScreen.h"
#include "EVSEWifi.h"
#include "utils.h"

void EVSEMenu::buildMenuItems() {
    uint8_t m = 0;

    MenuItems[m++] = MENU_CONFIG;
    if (evseController.config == CONFIG_SOCKET) {
        MenuItems[m++] = MENU_LOCK;
    }

    MenuItems[m++] = MENU_MODE;
    // ? Solar mode and Load Balancing Disabled/Master?
    if (evseController.mode == MODE_SOLAR && evseModbus.amIMasterOrDisabled()) {
        MenuItems[m++] = MENU_START;
        MenuItems[m++] = MENU_STOP;
        MenuItems[m++] = MENU_IMPORT;
    }

    MenuItems[m++] = MENU_LOADBL;
    // ? Mode Smart/Solar and Load Balancing Disabled/Master?
    if (evseController.mode != MODE_NORMAL && evseModbus.amIMasterOrDisabled()) {
        MenuItems[m++] = MENU_MAINS;
    }
    // ? Mode Smart/Solar or LoadBl Master?
    if (evseController.mode != MODE_NORMAL && (evseModbus.amIMasterOrDisabled() || evseModbus.isLoadBalancerMaster())) {
        MenuItems[m++] = MENU_MIN;
    }

    if (evseModbus.isLoadBalancerMaster()) {
        MenuItems[m++] = MENU_CIRCUIT;
    }

    MenuItems[m++] = MENU_MAX;
    MenuItems[m++] = MENU_SWITCH;
    MenuItems[m++] = MENU_RCMON;
    MenuItems[m++] = MENU_RFIDREADER;

    // ? Smart or Solar mode?
    if (evseController.mode != MODE_NORMAL) {
        if (evseModbus.amIMasterOrDisabled()) {
            MenuItems[m++] = MENU_MAINSMETER;
            // - - ? Sensorbox?
            if (evseModbus.mainsMeter == MM_SENSORBOX) {
                if (evseModbus.isGridActive()) {
                    MenuItems[m++] = MENU_GRID;
                }

                if (evseModbus.isCalibrationActive()) {
                    MenuItems[m++] = MENU_CALIBRATION;
                }
            } else if (evseModbus.mainsMeter != MAINS_METER_DISABLED) {
                MenuItems[m++] = MENU_MAINSMETERADDRESS;
                MenuItems[m++] = MENU_MAINSMETERMEASURE;

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
        // - ? EV meter configured?
        if (evseModbus.evMeter != EV_METER_DISABLED) {
            MenuItems[m++] = MENU_EVMETERADDRESS;
        }

        if (evseModbus.amIMasterOrDisabled()) {
            // ? Custom electric meter used?
            if (evseModbus.mainsMeter == MM_CUSTOM || evseModbus.pvMeter == MM_CUSTOM || evseModbus.evMeter == MM_CUSTOM) {
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

    MenuItems[m++] = MENU_WIFI;
    MenuItems[m++] = MENU_MAX_TEMPERATURE;
    MenuItems[m++] = MENU_LEDS;
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
        case MENU_START:
            return evseController.solarStartCurrent;
        case MENU_STOP:
            return evseController.solarStopTime;
        case MENU_IMPORT:
            return evseController.solarImportCurrent;
        case MENU_LOADBL:
            return evseModbus.getLoadBl();
        case MENU_MAINS:
            return evseController.maxMains;
        case MENU_MIN:
            return evseController.minCurrent;
        case MENU_MAX:
            return evseController.maxCurrent;
        case MENU_CIRCUIT:
            return evseModbus.maxCircuit;
        case MENU_LOCK:
            return evseLockActuator.getLockType();
        case MENU_SWITCH:
            return evseController.externalSwitch;
        case MENU_RCMON:
            return evseController.RCmon;
        case MENU_CALIBRATION:
            return evseController.ICal;
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

        // Status writeable
        case NODE_STATUS_STATE:
            return evseController.state;
        case NODE_STATUS_ERROR:
            return evseController.errorFlags;
        case NODE_STATUS_CURRENT:
            //return evseModbus.balancedCurrent[0];
            return evseController.chargeCurrent;
        case NODE_STATUS_SOLAR_TIMER:
            return evseController.solarStopTimer;
        case NODE_STATUS_ACCESS:
            return evseRFID.rfidAccessBit;
        case NODE_STATUS_CONFIG_CHANGED:
            return evseController.statusConfigChanged;
        // Status readonly
        case NODE_STATUS_MAX:
            return evseController.cableMaxCapacity;
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
            // Do not change Charge Mode when set to Normal or Load Balancing is disabled
            if (evseController.mode == MODE_NORMAL || evseModbus.isLoadBalancerDisabled()) {
                break;
            }
        case MENU_MODE:
            evseController.mode = val;
            break;
        case MENU_START:
            evseController.solarStartCurrent = val;
            break;
        case MENU_STOP:
            evseController.solarStopTime = val;
            break;
        case MENU_IMPORT:
            evseController.solarImportCurrent = val;
            break;
        case MENU_LOADBL:
            evseModbus.configureModbusMode(val);
            evseModbus.setLoadBl(val);
            break;
        case MENU_MAINS:
            evseController.maxMains = val;
            break;
        case MENU_MIN:
            evseController.minCurrent = val;
            break;
        case MENU_MAX:
            evseController.maxCurrent = val;
            break;
        case MENU_CIRCUIT:
            evseModbus.maxCircuit = val;
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
        case MENU_CALIBRATION:
            evseController.ICal = val;
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
            return evseController.maxTemperature = val;
        case MENU_LEDS:
            return evseRgbLeds.ledsEnabled = val != 0;

        // Status writeable
        case NODE_STATUS_STATE:
            evseController.setState(val);
            break;

        case NODE_STATUS_ERROR:
            evseController.onNodeReceivedError(val);
            break;

        case NODE_STATUS_CURRENT:
            evseModbus.setOverrideCurrent(val);
            break;

        case NODE_STATUS_SOLAR_TIMER:
            evseController.setSolarStopTimer(val);
            break;

        case NODE_STATUS_ACCESS:
            if (val == 0 || val == 1) {
                evseController.setAccess(val);
            }
            break;

        case NODE_STATUS_CONFIG_CHANGED:
            evseController.statusConfigChanged = val;
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

        case MENU_START:
            sprintf(Str, I18N_MENUSTART_FORMAT, value);
            return Str;

        case MENU_STOP:
            if (value) {
                sprintf(Str, I18N_MENUSTOP_FORMAT, value);
                return Str;
            } else
                return i18nStrDisabled;
        case MENU_LOADBL:
            /*if (controller->externalMaster && value == 1)
                return "Node 0";
            else*/
            return i18nStrLoadBl[evseModbus.getLoadBl()];
        case MENU_MAINS:
        case MENU_MIN:
        case MENU_MAX:
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
        case MENU_LEDS:
            return i18nStrLeds[evseRgbLeds.ledsEnabled ? 1 : 0];
        case MENU_EXIT:
            return i18nStrExitMenu;
        default:
            return "";
    }
}

/**
 * Counts nr of menu options currently available
 */
uint8_t EVSEMenu::getPosInMenu() {
    for (uint8_t i = 0; i < menuItemsCount; i++) {
        if (MenuItems[i] == currentMenuOption)
            return i + 1u;
    }

    return 0;
}

/**
 * Navigate left/right in a menu of int options
 */
unsigned int EVSEMenu::circleValues(uint8_t Buttons, unsigned int Value, unsigned int minValue, unsigned int maxValue) {
    switch (Buttons) {
        case BUTTON_RIGHT_MASK:
            return (Value >= maxValue) ? minValue : Value++;

        case BUTTON_LEFT_MASK:
            return (Value <= minValue) ? maxValue : Value--;

        default:
            return Value;
    }
}

/**
 * Navigate left/right in a menu of char array options
 */
unsigned char EVSEMenu::prevNextMenuEntry(uint8_t Buttons, unsigned char Value) {
    buildMenuItems();

    unsigned int posInMenu;
    for (posInMenu = 0; posInMenu < menuItemsCount; posInMenu++) {
        if (Value == MenuItems[posInMenu])
            break;
    }

    posInMenu = circleValues(Buttons, posInMenu, 0, menuItemsCount - 1u);

    return MenuItems[posInMenu];
}

void EVSEMenu::handleButtonsReleased() {
    // Button 2 released before entering menu
    if (currentMenuOption == MENU_ENTER) {
        currentMenuOption = MENU_NO_OPTION;
    }

    buttonRelease = 0;
    buttonRepeat = 0;
    // debounce keys (blocking)
    delay(10);
}

void EVSEMenu::handleButtonOPressed() {
    switch (currentMenuOption) {
        case MENU_NO_OPTION:
            // Button 2 just pressed, must hold for 2 seconds to enter menu
            if (buttonRelease == 0) {
                currentMenuOption = MENU_ENTER;
                buttonTimer = millis() + 2000;
            }
            break;

        case MENU_ENTER:
            // Button o was pressed for 2 seconds, then we enter the menu
            if (millis() >= buttonTimer) {
                currentMenuOption = MENU_CONFIG;
                buttonRelease = 1;
            }
            break;

        case MENU_CALIBRATION:
            if (buttonRelease != 0) {
                return;
            }

            if (subMenu) {
                evseController.calculateCalibration(CT1);
            } else {
                // make working copy of CT1 value
                CT1 = (unsigned int)abs(evseController.Irms[0]);
            }

            subMenu = subMenu ? 0 : 1;
            buttonRelease = 1;
            break;

        default:
            if (buttonRelease != 0) {
                return;
            }

            buttonRelease = 1;
            // In submenu => exit submenu
            if (subMenu) {
                subMenu = 0;
                return;
            }

            // Entering a submenu
            subMenu = 1;
            // Exit Main Menu
            if (currentMenuOption == MENU_EXIT) {
                currentMenuOption = MENU_NO_OPTION;
                subMenu = 0;
                endInactivityTask();
                // Skip updating of the LCD
                buttonRelease = 2;
                updateSettings();
            }
    }
}

void EVSEMenu::updateSettings() {
    evseController.updateSettings();
    evseWifi.updateSettings();
    evseRFID.updateSettings();
    evseLockActuator.updateSettings();
    evseModbus.updateSettings();

    evseController.statusConfigChanged = 1;
}

void EVSEMenu::handleButtonBothArrowsPressed() {
    if (buttonRelease != 0) {
        return;
    }

    buttonRelease = 1;

    // Press both arrows buttons to reset
    if ((currentMenuOption == MENU_CALIBRATION) && subMenu) {
        // User in CT CAL submenu, reset Calibration value and exit submenu
        evseController.ICal = ICAL_DEFAULT;
        subMenu = 0;
        return;
    }

    // re-initialize LCD
    evseScreen.resetLCD();
}

void EVSEMenu::handleButtonArrowPressed(uint8_t Buttons) {
    switch (buttonRelease) {
        case 0:
            buttonRelease = 1;

            // We are circling between submenu options
            if (subMenu) {
                uint16_t value = 0;

                switch (currentMenuOption) {
                    case MENU_CALIBRATION:
                        CT1 = circleValues(Buttons, CT1, 100, 999);
                        break;

                    case MENU_EVMETER:
                        // do not display the Sensorbox here
                        value = getMenuItemValue(currentMenuOption);
                        do {
                            value = circleValues(Buttons, value, menuEntries[currentMenuOption].Min, menuEntries[currentMenuOption].Max);
                        } while (value == MM_SENSORBOX);
                        setMenuItemValue(currentMenuOption, value);
                        break;

                    default:
                        value = getMenuItemValue(currentMenuOption);
                        value = circleValues(Buttons, value, menuEntries[currentMenuOption].Min, menuEntries[currentMenuOption].Max);
                        setMenuItemValue(currentMenuOption, value);
                        break;
                }

                return;
            }

            // Move to next / prev menu entry
            currentMenuOption = prevNextMenuEntry(Buttons, currentMenuOption);
            break;

        case 2:
            // Repeat button after 0.5 second
            if (buttonRepeat == 0) {
                buttonRepeat = 500;
                buttonTimer = millis() + buttonRepeat;
            }
            break;

        default:
            // Repeat button if buttonTimer has passed
            if (buttonRepeat && millis() >= buttonTimer) {
                buttonRelease = 0;
                if (buttonRepeat > 1) {
                    buttonRepeat -= (buttonRepeat / 8);
                    buttonTimer = millis() + buttonRepeat;
                }
            }
    }
}

void EVSEMenu::handleButtons(uint8_t Buttons) {
    switch (Buttons) {
        case BUTTON_NONE_MASK:
            handleButtonsReleased();
            break;

        case BUTTON_O_MASK:
            handleButtonOPressed();
            break;

        case BUTTON_LEFT_AND_RIGHT_MASK:
            // Buttons < and > pressed in order to reset or re-calibrate
            handleButtonBothArrowsPressed();
            break;

        case BUTTON_LEFT_MASK:
        case BUTTON_RIGHT_MASK:
            handleButtonArrowPressed(Buttons);
            break;
    }
}

void EVSEMenu::checkFastModeSwitch(uint8_t Buttons) {
    // Fast mode swith between "Sma-Sol" by pressing left button outside menu
    if (evseController.externalSwitch == SWITCH_SMARTSOLAR_BUTTON) {
        if (currentMenuOption == MENU_NO_OPTION && Buttons == BUTTON_LEFT_MASK && leftbuttonTimer == 0 && evseController.mode != MODE_NORMAL &&
            evseModbus.amIMasterOrDisabled()) {
            evseController.switchModeSolarSmart();
            resetInactivityTimer();
            leftbuttonTimer = 5;
            return;
        }

        if (leftbuttonTimer > 0 && Buttons == BUTTON_NONE_MASK) {
            leftbuttonTimer--;
            return;
        }
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
        // Thread to exit menu after 120 seconds of inactivity
        xTaskCreate((TaskFunction_t)&onInactivityTaskHandle, "onInactivityTaskHandle", 1024, this, 1, &inactivityTaskHandle);
    }
}

void EVSEMenu::endInactivityTask() {
    if (inactivityTaskHandle != NULL) {
        vTaskDelete(inactivityTaskHandle);
        inactivityTaskHandle = NULL;
    }
}

void EVSEMenu::checkExitMenuOnInactivity() {
    // Exit Setup menu after 120 seconds, unless rfid submenu so it will not exit the menu when learning/deleting cards
    while (true) {
        // Clean-up, this case should never happen
        if (currentMenuOption == MENU_NO_OPTION) {
            endInactivityTask();
            return;
        }

        if (inactivityTimer > MENU_INACTIVITY_TIMEOUT_SECONDS && !(currentMenuOption == MENU_RFIDREADER && subMenu)) {
            endInactivityTask();

            currentMenuOption = MENU_NO_OPTION;
            // don't save, but restore settings
            evseController.resetSettings();
            evseWifi.resetSettings();
            evseModbus.resetSettings();
            evseRFID.resetSettings();
            evseLockActuator.resetSettings();
            return;
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

/**
 * Called when one of the SmartEVSE buttons is pressed
 *
 * @param Buttons: < o >
 *          Value: 1 2 4
 *            Bit: 0:Pressed / 1:Released
 */
void EVSEMenu::onButtonChanged(uint8_t Buttons) {
    resetInactivityTimer();
    handleButtons(Buttons);

    if (buttonRelease == 1 || currentMenuOption == MENU_ENTER) {
        // Set value to 2, so that LCD will be updated only once
        buttonRelease = 2;
    }

    checkFastModeSwitch(Buttons);
    beginInactivityTask();
}

bool EVSEMenu::shouldRedrawMenu() {
    return (buttonRelease == 1 || currentMenuOption == MENU_ENTER || (currentMenuOption == MENU_RFIDREADER && subMenu) ||
            (currentMenuOption == MENU_WIFI && subMenu));
}

void EVSEMenu::setup() {
    buildMenuItems();
};

EVSEMenu evseMenu;