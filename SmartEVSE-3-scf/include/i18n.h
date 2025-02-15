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

#ifndef __I18N_H
#define __I18N_H

#include <Arduino.h>

// Text
const char i18nStrFixed[] = "Fixed";
const char i18nStrSocket[] = "Socket";
const char i18nStrSmart[] = "Smart";
const char i18nStrNormal[] = "Normal";
const char i18nStrSolar[] = "Solar";
const char i18nStrSolenoid[] = "Solenoid";
const char i18nStrMotor[] = "Motor";
const char i18nStrDisabled[] = "Disabled";
const char i18nStrLoadBl[9][9] = {"Disabled", "Master", "Node 1", "Node 2", "Node 3",
                                  "Node 4",   "Node 5", "Node 6", "Node 7"};
const char i18nStrSwitch[5][10] = {"Disabled", "Access B", "Access S", "Sma-Sol B", "Sma-Sol S"};
const char i18nStrGrid[2][10] = {"4Wire", "3Wire"};
const char i18nStrEnabled[] = "Enabled";
const char i18nStrExitMenu[] = "MENU";
const char i18nStrMainsAll[] = "All";

const char i18nStrEmCustomFunction3[] = "3:Hold. Reg";
const char i18nStrEmCustomFunction4[] = "4:Input. Reg";

const char i18nStrMainsHomeEVSE[] = "Home+EVSE";
const char i18nStrRFIDReader[6][10] = {"Disabled", "EnableAll", "EnableOne", "Learn", "Delete", "DeleteAll"};
const char i18nStrRFIDStatus[8][20] = {"Ready",
                                       "Card present",
                                       "Card stored",
                                       "Card deleted",
                                       "Card already stored",
                                       "Card not in storage",
                                       "Card storage full",
                                       "Invalid"};
const char i18nStrMeter[9][20] = {"Disabled", "Sensorbox", "Phoenix", "Finder", "Eastron",
                                  "ABB",      "SolarEdge", "WAGO",    "Custom"};
const char i18nStrWiFi[3][10] = {"Disabled", "Enabled", "SetupWifi"};
const char i18nStrLeds[2][10] = {"Disabled", "Enabled"};

const char i18nStrStateName[11][10] = {"A",      "B",         "C",        "D",  "COMM_B", "COMM_B_OK",
                                       "COMM_C", "COMM_C_OK", "Activate", "B1", "C1"};
const char i18nStrStateNameWeb[11][17] = {"Ready to Charge", "Connected to EV",  "Charging",        "D",
                                          "Request State B", "State B OK",       "Request State C", "State C OK",
                                          "Activate",        "Charging Stopped", "Stop Charging"};
const char i18nStrErrorNameWeb[9][20] = {"None",       "Not enough power", "Communication Error", "Temperature High",
                                         "Unused",     "RCM Tripped",      "Waiting for Solar",   "Test IO",
                                         "Flash Error"};

#define I18N_MENU_NOTINMENU "Not in menu"
#define I18N_MENU_HOLD2SEC "Hold 2 sec"
#define I18N_MENU_HOLDFORMENU "for Menu"

#define I18N_MENU_CONFIG "Fixed Cable or Type 2 Socket"
#define I18N_MENU_LOCK "Cable locking actuator type"
#define I18N_MENU_MIN "MIN Charge Current the EV will accept"
// #define I18N_MENU_MAX "MAX Charge Current for this EVSE (per phase)"
#define I18N_MENU_MAX "MAX Charge Current EVSE will dispatch"
#define I18N_MENU_LOADBL "Load Balancing mode for 2-8 SmartEVSEs"
#define I18N_MENU_SW "Switch function control on pin SW"
#define I18N_MENU_RCMON "Residual Current Monitor on pin RCM"
#define I18N_MENU_RFID "RFID reader, learn/remove cards"
#define I18N_MENU_EVEM "Type of EV electric meter"
#define I18N_MENU_EVAD "Address of EV electric meter"
#define I18N_MENU_MODE "Normal, Smart or Solar EVSE mode"
#define I18N_MENU_CIRCUIT "EVSE Circuit max Current"
#define I18N_MENU_GRID "Grid type to which the Sensorbox is connected"
#define I18N_MENU_CAL "Calibrate CT1 (CT2+3 will also change)"
#define I18N_MENU_MAINS "Max MAINS Current (usually contracted power)"
#define I18N_MENU_START "Surplus energy start Current"
#define I18N_MENU_STOP "Stop solar charging at 6A after this time"
#define I18N_MENU_IMPORT "Grid import when solar charging"
#define I18N_MENU_MAINEM "Type of mains electric meter"
#define I18N_MENU_MAINAD "Address of mains electric meter"
#define I18N_MENU_MAINM "Mains electric meter scope (What does it measure?)"
#define I18N_MENU_PVEM "Type of PV electric meter"
#define I18N_MENU_PVAD "Address of PV electric meter"
#define I18N_MENU_EMBO "Byte order of custom electric meter"
#define I18N_MENU_EMDATA "Data type of custom electric meter"
#define I18N_MENU_EMFUNC "Modbus Function of custom electric meter"
#define I18N_MENU_EMUREG "Register for Voltage (V) of custom electric meter"
#define I18N_MENU_EMUDIV "Divisor for Voltage (V) of custom electric meter"
#define I18N_MENU_EMIREG "Register for Current (A) of custom electric meter"
#define I18N_MENU_EMIDIV "Divisor for Current (A) of custom electric meter"
#define I18N_MENU_EMPREG "Register for Power (W) of custom electric meter"
#define I18N_MENU_EMPDIV "Divisor for Power (W) of custom electric meter"
#define I18N_MENU_EMEREG "Register for Energy (kWh) of custom electric meter"
#define I18N_MENU_EMEDIV "Divisor for Energy (kWh) of custom electric meter"
#define I18N_MENU_EMREAD "Max register read at once of custom electric meter"
#define I18N_MENU_WIFI "Connect to WiFi access point"
#define I18N_MENU_TEMPERATURE "Max operation temperature in degrees"
#define I18N_MENU_LEDS "Enable or disabled RGB leds"
#define I18N_MENU_EXIT "EXIT menu and save settings"

#define I18N_RFID_CARDSTORED "Card Stored"
#define I18N_RFID_CARDALREADY "Card already stored!"
#define I18N_RFID_CARDDELETED "Card Deleted"
#define I18N_RFID_CARDNOTFOUND "Card not in storage!"
#define I18N_RFID_STORAGEFULL "Card storage full!"

#define I18N_ERROR_NOSERIALCOM1 "ERROR NO"
#define I18N_ERROR_NOSERIALCOM2 "SERIAL COM"
#define I18N_ERROR_NOSERIALCOM3 "CHECK"
#define I18N_ERROR_NOSERIALCOM4 "WIRING"

#define I18N_ERROR_TEMP1 "HIGH TEMP"
#define I18N_ERROR_TEMP2 "ERROR"
#define I18N_ERROR_TEMP3 "CHARGING"
#define I18N_ERROR_TEMP4 "STOPPED"

#define I18N_ERROR_RCM1 "RESIDUAL"
#define I18N_ERROR_RCM2 "FAULT"
#define I18N_ERROR_RCM3 "CURRENT"
#define I18N_ERROR_RCM4 "DETECTED"
#define I18N_ERROR_RCM5 "PRESS"
#define I18N_ERROR_RCM6 "BUTTON"
#define I18N_ERROR_RCM7 "TO"
#define I18N_ERROR_RCM8 "RESET"

#define I18N_ERROR_OTA1 "BOOTLOADER"
#define I18N_ERROR_OTA2 "UPDATE ERR"

#define I18N_ERROR_LESS6A1 "WAITING"
#define I18N_ERROR_LESS6A2 "FOR POWER"

#define I18N_ERROR_NOSUN1 "WAITING"
#define I18N_ERROR_NOSUN2 "FOR SOLAR"

#define I18N_STATE_CHARGING "CHARGING"
#define I18N_STATE_CHARGING2 "%u.%u A"
#define I18N_STATE_CHARGINGPOWER "%u.%01u kW"
#define I18N_STATE_CHARGINGENERGY "%u.%02u kWh"
#define I18N_STATE_CHARGINGENERGY "%u.%02u kWh"

#define I18N_STATE_READY1 "READY TO"
#define I18N_STATE_READY1DELAY "READY %u"
#define I18N_STATE_READY1NODELAY "READY"
#define I18N_STATE_READY2 "CHARGE"
#define I18N_STATE_READY2DELAY "CHARGE %u"

#define I18N_STATE_READYRFIDON1 "PRESENT"
#define I18N_STATE_READYRFIDON2 "RFID CARD"

#define I18N_RFID_INVALID1 "INVALID"
#define I18N_RFID_INVALID2 "RFID CARD"

#define I18N_ACCESSDENIED1 "ACCESS"
#define I18N_ACCESSDENIED2 "DENIED"

#define I18N_MODE_SOLAR "SOLAR"
#define I18N_MODE_SMART "SMART"

#define I18N_SOLARSTOPTIMER_FORMAT "%02u:%02u"
#define I18N_POWERMEASURED_FORMAT_SHORT "%1u.%1ukW"
#define I18N_POWERMEASURED_FORMAT_LONG "%ukW"
#define I18N_POWERAMPS_FORMAT "%uA"
#define I18N_ENERGYCHARGED_FORMAT "%2u.%1ukWh"
// #define I18N_CURRENTS_FORMAT "%dA"
#define I18N_CURRENTS_FORMAT_SOLAR "%d.%uA"
#define I18N_CURRENTS_FORMAT_SMART "%u.%uA"
#define I18N_CURRENTS_FORMAT_SMART_SYMBOL "-%u.%uA"
#define I18N_TEMPERATURE_FORMAT "%i%cC"
#define I18N_MENUNAVIGATIONPOSITION_FORMAT "%u/%u"
#define I18N_SOFTWAREVERSION_FORMAT "v%s"

#define I18N_MENUSTART_FORMAT "-%2u A"
#define I18N_MENUSTOP_FORMAT "%2u min"
#define I18N_MENUIMPORT_FORMAT "%2u A"

#define I18N_WIFI_APHOSTNAME_PREFIX "SmartEVSE-"

const char* geti18nStrMeterText(uint8_t statusCode);

const char* geti18nRfidStatusText(uint8_t code);

const char* geti18nStateText(uint8_t stateCode);

const char* geti18nErrorText(uint8_t ErrorCode);

#endif
