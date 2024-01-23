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

#ifndef __EVSEWIFI
#define __EVSEWIFI

#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <time.h>

#define WIFI_MODE_DISABLED 0
#define WIFI_MODE_ENABLED 1
#define WIFI_MODE_START_PORTAL 2
#define PORTAL_START_WAIT_SECONDS 5
#define DEFAULT_AP_PASSWORD "00000000"

static const uint16_t WEB_SERVER_PORT = 80;
// Portal will be available 2 minutes to connect to, then close. (if connected
// within this time, it will remain active)
static const uint16_t CONFIG_PORTAL_TIMEOUT = 120;
// Use 0 => random channel from 1-13
static const uint8_t CONFIG_PORTAL_CHANNEL = 0;

static void WiFiStationGotIp(WiFiEvent_t event, WiFiEventInfo_t info);
static void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info);

class EVSEWifi {
   public:
    EVSEWifi(){};

    void setup();
    void setWifiMode(const uint8_t newMode);
    const uint8_t getWifiMode();
    const char* getApHostname();
    const char* getApPassword();
    void startwebServer();
    void stopwebServer();
    void startConfigPortal();
    uint16_t getNTPLocalTime();
    void updateSettings();
    void resetSettings();
    bool isPortalReady();
    uint8_t getPortalCountdownSeconds();
    IPAddress getLlocalIp();
    void onWiFiStationGotIp();

    uint8_t wifiMode = WIFI_MODE_DISABLED;

   protected:
    TaskHandle_t startPortalTaskHandle = NULL;
    static void startPortalTask();
    void endPortalTask();
    static void getSettings(AsyncWebServerRequest* request);
    static void postSettings(AsyncWebServerRequest* request);
    static void postReboot(AsyncWebServerRequest* request);

   private:
    String apHostname;
    String apPassword;
    IPAddress localIp;
    bool isBootLoader = true;
    char sprintfStr[255];

    DNSServer* dnsServer;
    AsyncWebServer* webServer;
    // AsyncWebSocket *asyncWebSocket;
    unsigned long startPortalTimer = 0;
    unsigned long ntpLocalTimeSync = 0;
    uint16_t lastTimeinfo = 0;

    void readEpromSettings();
    void writeEpromSettings();
    void wifiSetup();
    String genRandomAPpassword();
    uint32_t getMacId();
};

extern EVSEWifi evseWifi;

#endif