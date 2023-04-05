#include <Arduino.h>
#include <ArduinoJson.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include <Preferences.h>

#include <AsyncTCP.h>
#include <ESPAsync_WiFiManager.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <WiFi.h>

#include "EVSEController.h"
#include "EVSELogger.h"
#include "EVSEModbus.h"
#include "EVSEOTA.h"
#include "EVSERFID.h"
#include "EVSEWifi.h"
#include "i18n.h"
#include "main.h"

const char* PREFS_WIFI_NAMESPACE = "settings";
const char* PREFS_WIFI_MODE_KEY = "WIFImode";
const char* PREFS_WIFI_AP_PASSWORD = "APpassword";

// only one server is supported
const char* NTP_SERVER = "europe.pool.ntp.org";

// Specification of the Time Zone string:
// http://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
// list of time zones: https://remotemonitoringsystems.ca/time-zone-abbreviations.php
// Europe/Madrid
const char* TZ_INFO = "CET-1CEST,M3.5.0,M10.5.0/3";

// Out of class due to multiple definitions linked error
ESPAsync_WiFiManager* espAsync_wifiManager;

void EVSEWifi::enableWiFi() {
    dnsServer = new DNSServer();
    webServer = new AsyncWebServer(WEB_SERVER_PORT);
    espAsync_wifiManager = new ESPAsync_WiFiManager(webServer, dnsServer, apHostname.c_str());
    asyncWebSocket = new AsyncWebSocket("/ws");

    espAsync_wifiManager->setDebugOutput(true);
    espAsync_wifiManager->setConfigPortalChannel(CONFIG_PORTAL_CHANNEL);
    espAsync_wifiManager->setAPStaticIPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));

    // Portal will be available 2 minutes to connect to, then close. (if connected within this time, it will remain active)
    espAsync_wifiManager->setConfigPortalTimeout(CONFIG_PORTAL_TIMEOUT);

    // Start the mDNS responder so that the SmartEVSE can be accessed using a local hostame: http://SmartEVSE-xxxxxx.local
    if (!MDNS.begin(apHostname.c_str())) {
        EVSELogger::error("EVSEWifi:: Error setting up MDNS responder!");
    } else {
        EVSELogger::info("mDNS responder started. http://");
        EVSELogger::info(apHostname + ".local");
    }

    // WiFi.onEvent(onWifiDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    //  WiFi.onEvent(onWifiGotIp, ARDUINO_EVENT_WIFI_STA_GOT_IP);
    //  WiFi.onEvent(onWifiStop, SYSTEM_EVENT_AP_STOP);

    configTzTime(TZ_INFO, NTP_SERVER);
    // if(!getLocalTime(&timeinfo)) EVSELogger::info("Failed to obtain time");
}

void EVSEWifi::getSettings(AsyncWebServerRequest* request) {
    DynamicJsonDocument doc(1200);
    doc["version"] = String(EVSE_VERSION);

    doc["controller"]["vehicleConnected"] = evseController.isVehicleConnected();
    doc["controller"]["minCurrent"] = evseController.minCurrent;
    doc["controller"]["maxCurrent"] = evseController.maxCurrent;
    doc["controller"]["mode"] = evseController.mode;
    doc["controller"]["modeText"] = evseController.mode == MODE_SMART ? i18nStrSmart : (evseController.mode == MODE_SOLAR ? i18nStrSolar : i18nStrNormal);
    doc["controller"]["config"] = evseController.config;
    doc["controller"]["state"] = evseController.state;
    doc["controller"]["stateText"] = geti18nStateText(evseController.state);
    doc["controller"]["error"] = evseController.errorFlags;
    doc["controller"]["errorText"] = (evseController.errorFlags != 0) ? geti18nErrorText(evseController.errorFlags) : "";
    doc["controller"]["temperature"] = evseController.temperature;
    doc["controller"]["maxTemperature"] = evseController.maxTemperature;
    doc["controller"]["Irms"]["L1"] = evseController.Irms[0];
    doc["controller"]["Irms"]["L2"] = evseController.Irms[1];
    doc["controller"]["Irms"]["L3"] = evseController.Irms[2];
    doc["controller"]["cableMaxCapacity"] = evseController.cableMaxCapacity;
    doc["controller"]["maxMains"] = evseController.maxMains;
    doc["controller"]["chargeDelaySeconds"] = evseController.getChargeDelaySeconds();
    doc["controller"]["chargeCurrent"] = evseController.chargeCurrent;
    doc["controller"]["solarStopTimer"] = evseController.solarStopTimer;
    doc["controller"]["solarStartCurrent"] = evseController.solarStartCurrent;
    doc["controller"]["solarImportCurrent"] = evseController.solarImportCurrent;

    doc["modbus"]["mainsMeterText"] = geti18nStrMeterText(evseModbus.mainsMeter);
    doc["modbus"]["evMeterText"] = geti18nStrMeterText(evseModbus.evMeter);
    doc["modbus"]["pvMeterText"] = geti18nStrMeterText(evseModbus.pvMeter);
    doc["modbus"]["lastCTResponseMillis"] = evseModbus.getLastCTResponse();
    doc["modbus"]["powerMeasured"] = evseModbus.powerMeasured;
    // in kWh, precision 1 decimal
    doc["modbus"]["evMeterEnergy"] = round(evseModbus.getEvMeterEnergy() / 100) / 10;
    // in kWh, precision 1 decimal
    doc["modbus"]["energyCharged"] = round(evseModbus.energyCharged / 100) / 10;
    doc["modbus"]["overrideCurrent"] = evseModbus.getOverrideCurrent();

    doc["rfid"]["reader"] = evseRFID.RFIDReader;
    doc["rfid"]["status"] = evseRFID.RFIDstatus;
    doc["rfid"]["statusText"] = evseRFID.isEnabled() ? geti18nRfidStatusText(evseRFID.RFIDstatus) : "";
    doc["rfid"]["accessBit"] = evseRFID.rfidAccessBit == RFID_ACCESS_GRANTED ? true : false;

    String json;
    serializeJson(doc, json);

    AsyncWebServerResponse* response = request->beginResponse(200, "application/json", json);
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
}

void EVSEWifi::postSettings(AsyncWebServerRequest* request) {
    DynamicJsonDocument doc(512);

    if (request->hasParam("mode")) {
        String mode = request->getParam("mode")->value();
        switch (mode.toInt()) {
            case MODE_NORMAL:
                evseController.setAccess(true);
                evseController.setMode(MODE_NORMAL);
                break;

            case MODE_SOLAR:
                evseController.setAccess(true);
                evseController.setMode(MODE_SOLAR);
                break;

            case MODE_SMART:
                evseController.setAccess(true);
                evseController.setMode(MODE_SMART);
                break;

            default:
                mode = "ERROR: Value not allowed!";
        }
        doc["mode"] = mode;
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

void EVSEWifi::startwebServer() {
    webServer->on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        EVSELogger::debug("page / (root) requested and sent");
        request->send(SPIFFS, "/index.html");
    });

    webServer->on("/update", HTTP_GET, EVSEOTA::updateGETRequestHandler);
    webServer->on("/update", HTTP_POST, EVSEOTA::updatePOSTRequestHandler, EVSEOTA::updateMultipartUploadHandler);

    webServer->on("/settings", HTTP_GET, EVSEWifi::getSettings);
    webServer->on("/settings", HTTP_POST, EVSEWifi::postSettings,
                  [](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {});

    webServer->on("/reboot", HTTP_POST, EVSEWifi::postReboot,
                  [](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {});

    webServer->serveStatic("/", SPIFFS, "/");
    webServer->onNotFound([](AsyncWebServerRequest* request) { request->send(404); });
    webServer->addHandler(asyncWebSocket);
    webServer->begin();

    EVSELogger::info("HTTP server started");
}

void EVSEWifi::stopwebServer() {
    asyncWebSocket->closeAll();
    webServer->end();
}

void EVSEWifi::startConfigPortal() {
    // blocking until connected or timeout
    espAsync_wifiManager->startConfigPortal(apHostname.c_str(), apPassword.c_str());
}

void EVSEWifi::endPortalTask() {
    if (startPortalTaskHandle != NULL) {
        vTaskDelete(startPortalTaskHandle);
        startPortalTaskHandle = NULL;
    }
}

void EVSEWifi::startPortalTask(EVSEWifi* myself) {
    // Wait 10 seconds before starting the portal
    vTaskDelay((PORTAL_START_WAIT_SECONDS * 1000) / portTICK_PERIOD_MS);

    if (WiFi.getMode() != WIFI_AP_STA) {
        EVSELogger::info("Start Portal...");
        myself->stopwebServer();
        // blocking until connected or timeout
        myself->startConfigPortal();
        // restart webserver
        myself->startwebServer();

        myself->setWifiMode(WIFI_MODE_ENABLED);
    }

    myself->startPortalTimer = 0;
    myself->endPortalTask();
}

void EVSEWifi::setWifiMode(const uint8_t mode) {
    if (wifiMode == mode) {
        return;
    }

    // Stop portal thread
    if (startPortalTaskHandle != NULL) {
        vTaskDelete(startPortalTaskHandle);
        startPortalTaskHandle = NULL;
    }

    switch (mode) {
        case WIFI_MODE_ENABLED:
            if (WiFi.getMode() == WIFI_OFF) {
                EVSELogger::info("Starting WiFi...");
                WiFi.mode(WIFI_STA);
                WiFi.begin();
                WiFi.setAutoReconnect(true);
            }
            break;

        case WIFI_MODE_DISABLED:
            if (WiFi.getMode() != WIFI_OFF) {
                EVSELogger::info("Stopping WiFi...");
                WiFi.disconnect(true);
            }
            break;

        case WIFI_MODE_START_PORTAL:
            // new thread to wait 10 seconds before starting the portal
            startPortalTimer = millis() + (PORTAL_START_WAIT_SECONDS * 1000);
            xTaskCreate((TaskFunction_t)&startPortalTask, "startPortalTask", 10000, this, 1, &startPortalTaskHandle);
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

bool EVSEWifi::isNTPLocalTimeAvailable() {
    // retrieve time from NTP server
    return getLocalTime(&timeinfo, 1000U);
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

struct tm EVSEWifi::getNTPLocalTime() {
    return timeinfo;
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
        EVSELogger::error("Unable to open preferences for EVSEWifi");
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

    if (preferences.begin("KeyStorage", true) == true) {
        uint32_t serialnr = preferences.getUInt("serialnr", 0);
        preferences.end();

        if (serialnr) {
            // overwrite APhostname if serialnr is programmed
            apHostname = I18N_WIFI_APHOSTNAME_PREFIX + String(serialnr & 0xffff, 10);
        }
    }
}

void EVSEWifi::writeEpromSettings() {
    Preferences preferences;
    if (preferences.begin(PREFS_WIFI_NAMESPACE, false) != true) {
        EVSELogger::error("Unable to write preferences for EVSEWifi");
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

    enableWiFi();
    startwebServer();
    setWifiMode(wifiMode);
}

EVSEWifi evseWifi;