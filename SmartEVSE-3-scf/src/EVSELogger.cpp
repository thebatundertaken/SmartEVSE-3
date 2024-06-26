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

#include "EVSELogger.h"

#include <Arduino.h>
#include "WebSerial.h"

uint8_t EVSELogger::LogLevel = LOG_LEVEL_WARN;
bool EVSELogger::webloggerEnabled = false;

void EVSELogger::error(const String& s) {
    Serial.println(s);
    if (webloggerEnabled) {
        WebSerial.println(s);
    }
}

void EVSELogger::warn(const String& s) {
    if (EVSELogger::LogLevel <= LOG_LEVEL_WARN) {
        Serial.println(s);
        if (webloggerEnabled) {
            WebSerial.println(s);
        }
    }
}

void EVSELogger::info(const String& s) {
    if (EVSELogger::LogLevel <= LOG_LEVEL_INFO) {
        Serial.println(s);
        if (webloggerEnabled) {
            WebSerial.println(s);
        }
    }
}

void EVSELogger::debug(const String& s) {
    if (EVSELogger::LogLevel <= LOG_LEVEL_DEBUG) {
        Serial.println(s);
        if (webloggerEnabled) {
            WebSerial.println(s);
        }
    }
}

void EVSELogger::enableWebserialLog(AsyncWebServer* server) {
    if (!webloggerEnabled) {
        webloggerEnabled = true;
        WebSerial.begin(server);
    }
}

void EVSELogger::disableWebserialLog() {
    if (webloggerEnabled) {
        webloggerEnabled = false;
        // WebSerial.end();
    }
}
