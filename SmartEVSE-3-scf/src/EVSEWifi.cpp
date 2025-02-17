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

#include "EVSEWifi.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsync_WiFiManager.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "EVSECluster.h"
#include "EVSEController.h"
#include "EVSELogger.h"
#include "EVSEModbus.h"
#include "EVSEOTA.h"
#include "EVSERFID.h"
#include "i18n.h"
#include "main.h"

const char* PREFS_WIFI_NAMESPACE = "settings";
const char* PREFS_WIFI_MODE_KEY = "WIFImode";
const char* PREFS_WIFI_AP_PASSWORD = "APpassword";
const char* PREFS_WIFI_KEYSTORAGE = "KeyStorage";

// only one server is supported
const char* NTP_SERVER = "europe.pool.ntp.org";

// Specification of the Time Zone string:
// http://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
// list of time zones:
// https://remotemonitoringsystems.ca/time-zone-abbreviations.php Europe/Madrid
const char* TZ_INFO = "CET-1CEST,M3.5.0,M10.5.0/3";

// Out of class due to multiple definitions linked error
ESPAsync_WiFiManager* espAsync_wifiManager;

void EVSEWifi::wifiSetup() {
    dnsServer = new DNSServer();
    webServer = new AsyncWebServer(WEB_SERVER_PORT);
    espAsync_wifiManager = new ESPAsync_WiFiManager(webServer, dnsServer, apHostname.c_str());
    // asyncWebSocket = new AsyncWebSocket("/ws");

    espAsync_wifiManager->setDebugOutput(true);
    espAsync_wifiManager->setConfigPortalChannel(CONFIG_PORTAL_CHANNEL);
    espAsync_wifiManager->setAPStaticIPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                                              IPAddress(255, 255, 255, 0));

    // Portal will be available 2 minutes to connect to, then close. (if connected
    // within this time, it will remain active)
    espAsync_wifiManager->setConfigPortalTimeout(CONFIG_PORTAL_TIMEOUT);
    localIp = WiFi.localIP();

    // Start the mDNS responder so that the SmartEVSE can be accessed using a
    // local hostame: http://SmartEVSE-xxxxxx.local
    if (!MDNS.begin(apHostname.c_str())) {
        EVSELogger::error("[EVSEWifi] Error setting up MDNS responder!");
    } else {
        sprintf(sprintfStr, "[EVSEWifi] mDNS responder started. http://%s.local", apHostname);
        EVSELogger::info(sprintfStr);
    }

    WiFi.onEvent(WiFiStationDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    WiFi.onEvent(WiFiStationGotIp, ARDUINO_EVENT_WIFI_STA_GOT_IP);
    //  WiFi.onEvent(onWifiStop, SYSTEM_EVENT_AP_STOP);

    configTzTime(TZ_INFO, NTP_SERVER);
}

/*void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  EVSELogger::debug("[EVSEWiFi] WiFi connection lost");
  // try to reconnect when not connected to AP
  if (WiFi.getMode() != WIFI_AP_STA)
  {
    EVSELogger::debug("[EVSEWiFi] Trying to Reconnect");
    WiFi.begin();
  }
}*/

void WiFiStationGotIp(WiFiEvent_t event, WiFiEventInfo_t info) {
    evseWifi.onWiFiStationGotIp();
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    EVSELogger::debug("[EVSEWiFi] WiFi connection lost");
    // try to reconnect when not connected to AP
    if (WiFi.getMode() != WIFI_AP_STA) {
        EVSELogger::debug("[EVSEWiFi] Trying to reconnect");
        // WiFi.begin();
    }
}

void EVSEWifi::onWiFiStationGotIp() {
    localIp = WiFi.localIP();
    sprintf(sprintfStr, "[EVSEWifi] Connected to AP: %s; Local IP: %s", WiFi.SSID(), localIp);
    EVSELogger::info(sprintfStr);
}

IPAddress EVSEWifi::getLlocalIp() {
    return localIp;
}

void EVSEWifi::getSettings(AsyncWebServerRequest* request) {
    DynamicJsonDocument doc(1200);
    doc["version"] = String(EVSE_VERSION);
    doc["hostname"] = String(evseWifi.apHostname);

    doc["controller"]["vehicleConnected"] = evseController.isVehicleConnected();
    doc["controller"]["isCharging"] = evseController.state == STATE_C_CHARGING;
    doc["controller"]["minEVCurrent"] = evseController.minEVCurrent;
    doc["controller"]["maxMains"] = evseController.maxMains;
    doc["controller"]["maxDeviceCurrent"] = evseController.maxDeviceCurrent;
    doc["controller"]["solarBoost"] = evseController.isSolarBoost();
    doc["controller"]["solarBoostCurrent"] = evseController.getSolarBoostCurrent();
    doc["controller"]["mode"] = evseController.mode;
    doc["controller"]["modeText"] = evseController.mode == MODE_SMART
                                        ? i18nStrSmart
                                        : (evseController.mode == MODE_SOLAR ? i18nStrSolar : i18nStrNormal);
    doc["controller"]["config"] = evseController.config;
    doc["controller"]["state"] = evseController.state;
    doc["controller"]["stateText"] = geti18nStateText(evseController.state);
    doc["controller"]["error"] = evseController.errorFlags;
    doc["controller"]["errorText"] =
        (evseController.errorFlags != 0) ? geti18nErrorText(evseController.errorFlags) : "";
    doc["controller"]["temperature"] = evseController.temperature;
    doc["controller"]["maxTemperature"] = evseController.maxTemperature;
    doc["controller"]["Irms"]["L1"] = evseController.Irms[0];
    doc["controller"]["Irms"]["L2"] = evseController.Irms[1];
    doc["controller"]["Irms"]["L3"] = evseController.Irms[2];
    doc["controller"]["cableMaxCapacity"] = evseController.getCableMaxCapacity();
    doc["controller"]["chargeDelaySeconds"] = evseController.getChargeDelaySeconds();
    doc["controller"]["chargeCurrent"] = evseController.getChargeCurrent();
    doc["controller"]["solarStopTimeMinutes"] = evseController.solarStopTimeMinutes;
    doc["controller"]["solarStartCurrent"] = evseController.solarStartCurrent;
    doc["controller"]["solarImportCurrent"] = evseController.solarImportCurrent;
    doc["controller"]["solarStopTimer"] = evseController.solarStopTimer;
    uint16_t localTime = evseWifi.getNTPLocalTime();
    doc["controller"]["time"] = localTime == UINT16_MAX ? -1 : localTime;
    if (evseController.isOperatingHoursEnabled()) {
        doc["controller"]["switchOnTime"] = evseController.getOperatingHoursOnTime();
        doc["controller"]["switchOffTime"] = evseController.getOperatingHoursOffTime();
    } else {
        doc["controller"]["switchOnTime"] = -1;
        doc["controller"]["switchOffTime"] = -1;
    }

    doc["modbus"]["mainsMeterText"] = geti18nStrMeterText(evseModbus.mainsMeter);
    doc["modbus"]["evMeterText"] = geti18nStrMeterText(evseModbus.evMeter);
    doc["modbus"]["pvMeterText"] = geti18nStrMeterText(evseModbus.pvMeter);
    doc["modbus"]["lastCTResponseMillis"] = evseModbus.getLastCTResponse();
    doc["modbus"]["powerMeasured"] = evseModbus.powerMeasured;
    // in kWh, precision 1 decimal
    doc["modbus"]["evMeterEnergy"] = round(evseModbus.getEvMeterEnergy() / 100) / 10;
    // in kWh, precision 1 decimal
    doc["modbus"]["energyCharged"] = round(evseModbus.energyCharged / 100) / 10;
    doc["modbus"]["overrideCurrent"] = evseCluster.getOverrideCurrent();

    doc["rfid"]["reader"] = evseRFID.RFIDReader;
    doc["rfid"]["status"] = evseRFID.RFIDstatus;
    doc["rfid"]["statusText"] = evseRFID.isEnabled() ? geti18nRfidStatusText(evseRFID.RFIDstatus) : "";
    doc["rfid"]["accessBit"] = evseRFID.rfidAccessBit == RFID_ACCESS_GRANTED ? true : false;

    doc["nerd"]["controller"]["CP"] = evseController.getControlPilot();
    doc["nerd"]["maxCurrentAvailable"] = evseController.getMaxCurrentAvailable();
    doc["nerd"]["errorFlags"] = evseController.errorFlags;
    doc["nerd"]["chargeDelaySeconds"] = evseController.errorFlags;
    doc["nerd"]["logLevel"] = EVSELogger::LogLevel;
    doc["nerd"]["clusterCurrent"] = evseCluster.getClusterCurrent();
    doc["nerd"]["DEBUG_LAST_CHARGE_REBALANCE_CALC"] = evseCluster.DEBUG_LAST_CHARGE_REBALANCE_CALC;

    String json;
    serializeJson(doc, json);

    AsyncWebServerResponse* response = request->beginResponse(200, "application/json", json);
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
}

void EVSEWifi::postSettings(AsyncWebServerRequest* request) {
    DynamicJsonDocument doc(512);
    bool updateSettings = false;

    if (request->hasParam("mode")) {
        String mode = request->getParam("mode")->value();
        updateSettings = true;
        switch (mode.toInt()) {
            case MODE_NORMAL:
                evseController.setAccess(true);
                evseController.switchMode(MODE_NORMAL);
                break;

            case MODE_SOLAR:
                evseController.setAccess(true);
                evseController.switchMode(MODE_SOLAR);
                break;

            case MODE_SMART:
                evseController.setAccess(true);
                evseController.switchMode(MODE_SMART);
                break;

            default:
                updateSettings = false;
                mode = "ERROR: Value not allowed!";
        }
        doc["mode"] = mode;
    }

    if (request->hasParam("loglevel")) {
        String value = request->getParam("loglevel")->value();
        uint8_t logLevel = _max((uint8_t)value.toInt(), LOG_LEVEL_DEBUG);
        logLevel = _min(logLevel, LOG_LEVEL_ERROR);
        EVSELogger::LogLevel = logLevel;
        doc["mode"] = "OK";
    }

    if (request->hasParam("resetFlags")) {
        evseController.errorFlags = ERROR_FLAG_NO_ERROR;
        doc["mode"] = "OK";
    }
    if (request->hasParam("maxmains")) {
        int maxMainsVal = request->getParam("maxmains")->value().toInt();
        if (maxMainsVal >= evseController.minEVCurrent && maxMainsVal <= evseController.maxDeviceCurrent) {
            evseController.maxMains = maxMainsVal;
            updateSettings = true;
            doc["mode"] = "OK";
        } else {
            doc["mode"] = "ERROR: Value not allowed!";
        }
    }

    if (request->hasParam("disableOperatingHours")) {
        evseController.disableOperatingHours();
        doc["mode"] = "OK";
    }

    if (request->hasParam("enableOperatingHours")) {
        int onTime = request->getParam("switchon")->value().toInt();
        int offTime = request->getParam("switchoff")->value().toInt();
        if (onTime >= 0 && onTime <= 2359 && offTime >= 0 && offTime <= 2359 && onTime != offTime) {
            evseController.setOperatingHours(onTime, offTime);
            doc["mode"] = "OK";
        } else {
            doc["mode"] = "ERROR: Value not allowed!";
        }
    }

    if (request->hasParam("solarBoost")) {
        bool active = request->getParam("solarBoost")->value().toInt() == 1;
        evseController.setSolarBoost(active);
        updateSettings = true;
        doc["mode"] = "OK";
    }

    if (updateSettings) {
        evseController.updateSettings();
    }

    String json;
    serializeJson(doc, json);

    request->send(200, "application/json", json);
}

void EVSEWifi::postReboot(AsyncWebServerRequest* request) {
    ESP.restart();

    DynamicJsonDocument doc(200);
    doc["reboot"] = true;

    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
}

void EVSEWifi::forceDisconnect(AsyncWebServerRequest* request) {
    DynamicJsonDocument doc(200);
    evseController.onDisconnectInProgress();
    evseController.setState(STATE_DISCONNECT_IN_PROGRESS);
    doc["forceDisconnect"] = true;

    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
}

void EVSEWifi::forceCharge(AsyncWebServerRequest* request) {
    DynamicJsonDocument doc(200);
    if (evseController.state != STATE_C_CHARGING) {
        evseController.onVehicleStartCharging();
        evseController.setState(STATE_C_CHARGING);
        doc["forceCharge"] = true;
    } else {
        doc["forceCharge"] = false;
    }

    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
}

void EVSEWifi::startwebServer() {
    webServer->on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        EVSELogger::debug("[EVSEWiFi] page / (root) requested and sent");
        request->send(SPIFFS, "/index.html");
    });

    webServer->on("/update", HTTP_GET, EVSEOTA::updateGETRequestHandler);
    webServer->on("/update", HTTP_POST, EVSEOTA::updatePOSTRequestHandler, EVSEOTA::updateMultipartUploadHandler);

    webServer->on("/settings", HTTP_GET, EVSEWifi::getSettings);
    webServer->on(
        "/settings", HTTP_POST, EVSEWifi::postSettings,
        [](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {});

    webServer->on(
        "/reboot", HTTP_POST, EVSEWifi::postReboot,
        [](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {});

    webServer->on(
        "/forcecharge", HTTP_GET, EVSEWifi::forceCharge,
        [](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {});

    webServer->on(
        "/forcedisconnect", HTTP_GET, EVSEWifi::forceDisconnect,
        [](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {});

    webServer->serveStatic("/", SPIFFS, "/");
    webServer->onNotFound([](AsyncWebServerRequest* request) { request->send(404); });

    EVSELogger::enableWebserialLog(webServer);

    webServer->begin();

    EVSELogger::info("[EVSEWiFi] HTTP server started");
}

void EVSEWifi::stopwebServer() {
    // asyncWebSocket->closeAll();
    webServer->end();
}

void EVSEWifi::startConfigPortal() {
    if (wifiMode == WIFI_MODE_START_PORTAL && WiFi.getMode() != WIFI_AP_STA) {
        evseWifi.stopwebServer();
        // blocking until connected or timeout
        espAsync_wifiManager->startConfigPortal(apHostname.c_str(), apPassword.c_str());
        evseWifi.wifiMode = WIFI_MODE_ENABLED;
        evseWifi.writeEpromSettings();
        // restart webserver
        evseWifi.startwebServer();

        // Force disconnect and re-connect
        evseWifi.setWifiMode(WIFI_MODE_DISABLED);
        evseWifi.setWifiMode(WIFI_MODE_ENABLED);
    }
}

void EVSEWifi::endPortalTask() {
    if (startPortalTaskHandle != NULL) {
        vTaskDelete(startPortalTaskHandle);
        startPortalTaskHandle = NULL;
    }
}

void EVSEWifi::startPortalTask() {
    // Wait 5 seconds before starting the portal
    vTaskDelay((PORTAL_START_WAIT_SECONDS * 1000) / portTICK_PERIOD_MS);

    EVSELogger::info("[EVSEWiFi] Starting wifi config portal...");
    evseWifi.startConfigPortal();
}

void EVSEWifi::setWifiMode(const uint8_t newMode) {
    char buffer[50];
    sprintf(buffer, "[EVSEWifi] Setting wifi mode from %u to %u", wifiMode, newMode);
    EVSELogger::info(buffer);

    wifiMode = newMode;

    // Stop portal thread
    endPortalTask();

    switch (newMode) {
        case WIFI_MODE_ENABLED:
            sprintf(sprintfStr, "[EVSEWifi] WiFi.getMode() = %u", WiFi.getMode());
            EVSELogger::debug(sprintfStr);

            // On boot WiFi.getMode() says it is connected, but it is not
            if (isBootLoader || WiFi.getMode() == WIFI_OFF) {
                EVSELogger::info("[EVSEWifi] Starting WiFi...");
                WiFi.mode(WIFI_STA);
                WiFi.begin();
                WiFi.setAutoReconnect(true);
            }
            break;

        case WIFI_MODE_DISABLED:
            if (WiFi.getMode() != WIFI_OFF) {
                EVSELogger::info("[EVSEWiFi] Stopping WiFi...");
                WiFi.disconnect(true);
            }
            break;

        case WIFI_MODE_START_PORTAL:
            if (WiFi.getMode() != WIFI_AP_STA) {
                if (startPortalTaskHandle != NULL) {
                    EVSELogger::warn("[EVSEWiFi] Race condition detected. Portal not started");
                    break;
                }

                EVSELogger::info("[EVSEWifi] Starting portal task...");
                // new thread to wait 5 seconds before starting the portal
                startPortalTimer = millis() + (PORTAL_START_WAIT_SECONDS * 1000);
                xTaskCreate((TaskFunction_t)&startPortalTask, "startPortalTask", 16384, NULL, 1,
                            &startPortalTaskHandle);
            }
            break;
    }
}

const uint8_t EVSEWifi::getWifiMode() {
    return wifiMode;
}

const char* EVSEWifi::getApHostname() {
    return apHostname.c_str();
}

const char* EVSEWifi::getApPassword() {
    return apPassword.c_str();
}

bool EVSEWifi::isPortalReady() {
    return wifiMode == WIFI_MODE_START_PORTAL && WiFi.getMode() == WIFI_AP_STA;
}

uint8_t EVSEWifi::getPortalCountdownSeconds() {
    unsigned long now = millis();
    if (now > startPortalTimer) {
        return 0;
    }

    return (int)((startPortalTimer - now) / 1000);
}

uint16_t EVSEWifi::getNTPLocalTime() {
    // Cache local time for 30 seconds
    if (ntpLocalTimeSync != 0 && ((millis() - ntpLocalTimeSync) <= 30000)) {
        return lastTimeinfo;
    }

    if (WiFi.status() != WL_CONNECTED) {
        return UINT16_MAX;
    }

    // retrieve time from NTP server
    ntpLocalTimeSync = millis();
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 1000U)) {
        lastTimeinfo = (timeinfo.tm_hour * 100) + timeinfo.tm_min;
    } else {
        EVSELogger::error("[EVSEWiFi] Failed to obtain local time");
        lastTimeinfo = UINT16_MAX;
    }

    return lastTimeinfo;
}

// read Mac, and reverse to ID
uint32_t EVSEWifi::getMacId() {
    uint32_t id = 0;

    for (int i = 0; i < 17; i = i + 8) {
        id |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }
    return id >> 2;  // low two bits are unused.
}

String EVSEWifi::genRandomAPpassword() {
    String password = "00000000";
    uint8_t i, c;
    for (i = 0; i < 8; i++) {
        c = random(16) + '0';
        if (c > '9')
            c += 'a' - '9' - 1;
        password[i] = c;
    }

    return password;
}

void EVSEWifi::readEpromSettings() {
    // SmartEVSE access point Name = SmartEVSE-xxxxx;
    apHostname = I18N_WIFI_APHOSTNAME_PREFIX + String(getMacId() & 0xffff, 10);

    // serialnr = preferences.begin("KeyStorage", true).getUInt("serialnr");

    wifiMode = WIFI_MODE_DISABLED;
    apPassword = DEFAULT_AP_PASSWORD;

    Preferences preferences;
    if (preferences.begin(PREFS_WIFI_NAMESPACE, true) != true) {
        EVSELogger::error("[EVSEWiFi] Unable to open preferences for EVSEWifi");
        return;
    }

    bool firstRun = true;
    if (preferences.isKey(PREFS_WIFI_MODE_KEY)) {
        firstRun = false;

        wifiMode = preferences.getUChar(PREFS_WIFI_MODE_KEY, WIFI_MODE_DISABLED);
        apPassword = preferences.getString(PREFS_WIFI_AP_PASSWORD, DEFAULT_AP_PASSWORD);
    }
    preferences.end();

    if (firstRun) {
        wifiMode = WIFI_MODE_DISABLED;
        apPassword = genRandomAPpassword();
        writeEpromSettings();
    }

    if (preferences.isKey(PREFS_WIFI_KEYSTORAGE)) {
        if (preferences.begin(PREFS_WIFI_KEYSTORAGE, true) == true) {
            uint32_t serialnr = preferences.getUInt("serialnr", 0);
            preferences.end();

            if (serialnr) {
                // overwrite APhostname if serialnr is programmed
                apHostname = I18N_WIFI_APHOSTNAME_PREFIX + String(serialnr & 0xffff, 10);
            }
        }
    } else {
        EVSELogger::info("[EVSEWifi] Keystorage not present");
    }
}

void EVSEWifi::writeEpromSettings() {
    Preferences preferences;
    if (preferences.begin(PREFS_WIFI_NAMESPACE, false) != true) {
        EVSELogger::error("[EVSEWiFi] Unable to write preferences for EVSEWifi");
        return;
    }

    preferences.putUChar(PREFS_WIFI_MODE_KEY, wifiMode);
    preferences.putString(PREFS_WIFI_AP_PASSWORD, apPassword);

    preferences.end();
}

void EVSEWifi::updateSettings() {
    writeEpromSettings();
}

void EVSEWifi::resetSettings() {
    readEpromSettings();
}

void EVSEWifi::setup() {
    readEpromSettings();

    wifiSetup();
    startwebServer();
    setWifiMode(wifiMode);
    isBootLoader = false;
}

EVSEWifi evseWifi;