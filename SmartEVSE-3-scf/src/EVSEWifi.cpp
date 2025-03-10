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
#include "EVSENetwork.h"

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

#include "EVSEController.h"
#include "EVSELogger.h"
#include "EVSEOTA.h"
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

    // Manual reconnect due to WiFi.setAutoReconnect(true) limitations
    WiFi.onEvent(WiFiStationDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    WiFi.onEvent(WiFiStationGotIp, ARDUINO_EVENT_WIFI_STA_GOT_IP);

    configTzTime(TZ_INFO, NTP_SERVER);
}

void WiFiStationGotIp(WiFiEvent_t event, WiFiEventInfo_t info) {
    evseWifi.onWiFiStationGotIp();
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    evseWifi.onWiFiStationDisconnected();
}

void EVSEWifi::onWiFiStationDisconnected() {
    if (wifiMode == WIFI_OFF) {
        EVSELogger::info("[EVSEWiFi] WiFi disabled");
        return;
    }

    // try to reconnect when not connected to AP
    if (wifiMode == WIFI_MODE_START_PORTAL) {
        EVSELogger::info("[EVSEWiFi] Portal mode detected, no need to reconnect");
        return;
    }

    EVSELogger::info("[EVSEWiFi] WiFi connection lost");
    if (wifiReconnectTaskHandle != NULL) {
        EVSELogger::debug("[EVSEWiFi] Race condition detected on disconnect task");
        return;
    }

    EVSELogger::info("[EVSEWiFi] Starting WiFi reconnect task...");
    xTaskCreate((TaskFunction_t)&startWifiReconnectTask, "startWifiReconnectTask", 16384, NULL, 1,
                &wifiReconnectTaskHandle);
}

void EVSEWifi::onWiFiStationGotIp() {
    localIp = WiFi.localIP();
    sprintf(sprintfStr, "[EVSEWifi] Connected to AP: %s; Local IP: %s", WiFi.SSID(), localIp);
    EVSELogger::info(sprintfStr);
    endWifiReconnectTask();
}

IPAddress EVSEWifi::getLlocalIp() {
    return localIp;
}

void EVSEWifi::startwebServer() {
    webServer->on("/", HTTP_GET, [](AsyncWebServerRequest* request) { request->send(SPIFFS, "/index.html"); });

    webServer->on("/update", HTTP_GET, [](AsyncWebServerRequest* request) { request->send(SPIFFS, "/update.html"); });

    webServer->on("/update", HTTP_POST, EVSEOTA::updatePOSTRequestHandler, EVSEOTA::updateMultipartUploadHandler);

    webServer->on("/currents", HTTP_POST, EVSENetwork::postSensorboxReadings);

    webServer->on("/settings", HTTP_GET, EVSENetwork::getWebUIData);
    webServer->on(
        "/settings", HTTP_POST, EVSENetwork::postSettings,
        [](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {});

    webServer->on(
        "/reboot", HTTP_POST, EVSENetwork::postReboot,
        [](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {});

    webServer->on(
        "/forcecharge", HTTP_GET, EVSENetwork::forceCharge,
        [](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {});

    webServer->on(
        "/forcedisconnect", HTTP_GET, EVSENetwork::forceDisconnect,
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

void EVSEWifi::startPortalTask() {
    // Wait 5 seconds before starting the portal
    vTaskDelay((PORTAL_START_WAIT_SECONDS * 1000) / portTICK_PERIOD_MS);

    EVSELogger::info("[EVSEWiFi] Starting wifi config portal...");
    evseWifi.startConfigPortal();
}

void EVSEWifi::endPortalTask() {
    if (startPortalTaskHandle != NULL) {
        vTaskDelete(startPortalTaskHandle);
        startPortalTaskHandle = NULL;
        delay(100);
    }
}

void EVSEWifi::startWifiReconnectTask() {
    uint32_t attempt = 1;
    uint32_t exponentialDelay = 15000;
    uint32_t maxDelay = 60000;
    char buffer[250];
    while (1) {
        if (evseWifi.wifiMode != WIFI_MODE_ENABLED) {
            EVSELogger::info("[EVSEWiFi] Wifi disabled, no need to reconnect");
            return;
        }

        if (WiFi.status() == WL_CONNECTED) {
            EVSELogger::info("[EVSEWiFi] Connected to Wifi, no need to reconnect");
            return;
        }

        sprintf(buffer, "[EVSEWiFi] Trying to reconnect [attempt %u]", attempt++);
        EVSELogger::info(buffer);

        WiFi.disconnect();
        WiFi.reconnect();

        exponentialDelay = _min(exponentialDelay + 1000, maxDelay);
        vTaskDelay(exponentialDelay / portTICK_PERIOD_MS);
    }

    EVSELogger::info("[EVSEWiFi] Reconnect loop done");
}

void EVSEWifi::endWifiReconnectTask() {
    if (wifiReconnectTaskHandle != NULL) {
        vTaskDelete(wifiReconnectTaskHandle);
        wifiReconnectTaskHandle = NULL;
        EVSELogger::info("[EVSEWiFi] Reconnect task deleted");
        delay(100);
    }
}

void EVSEWifi::setWifiMode(const uint8_t newMode) {
    char buffer[50];
    sprintf(buffer, "[EVSEWifi] Setting wifi mode from %u to %u", wifiMode, newMode);
    EVSELogger::info(buffer);

    wifiMode = newMode;

    // Stop portal and wifiReconnect threads
    endWifiReconnectTask();
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
                // setAutoReconnect fails if WiFi is down for several minutes (ex: router firmware update)
                // WiFi.setAutoReconnect(true);
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