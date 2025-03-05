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

#include "EVSEOTA.h"

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Update.h>

#include "EVSELogger.h"

void EVSEOTA::updatePOSTRequestHandler(AsyncWebServerRequest* request) {
    bool updateSuccessful = !Update.hasError();
    AsyncWebServerResponse* response =
        request->beginResponse(200, "text/plain", updateSuccessful ? "OK" : Update.errorString());
    response->addHeader("Connection", "close");
    request->send(response);

    if (updateSuccessful) {
        delay(500);
        ESP.restart();
    }
}

void EVSEOTA::updateMultipartUploadHandler(AsyncWebServerRequest* request,
                                           String filename,
                                           size_t index,
                                           uint8_t* data,
                                           size_t len,
                                           bool final) {
    char sprintChar[255];
    if (!index) {
        filename.toLowerCase();
        EVSELogger::info("\nUpdate Start:");
        EVSELogger::info(filename);
        if (filename.indexOf("spiffs") >= 0) {
            EVSELogger::info("[EVSEOTA] SPIFFS partition update in progress");
            if (!Update.begin(SPI_PARTITION_SIZE, U_SPIFFS)) {
                Update.printError(Serial);
            }
        } else if (filename.indexOf("firmware") >= 0 && !Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000),
                   U_FLASH) {
            EVSELogger::info("[EVSEOTA] Firmware update in progress");
            Update.printError(Serial);
        } else {
            sprintf(sprintChar, "[EVSEOTA] FAIL: Invalid filename '%s'", filename);
            EVSELogger::warn(sprintChar);
        }
    }

    if (!Update.hasError()) {
        if (Update.write(data, len) != len) {
            sprintf(sprintChar, "[EVSEOTA] FAIL: file wasn't written. %s", Update.errorString());
            EVSELogger::warn(sprintChar);
            Update.printError(Serial);
        } else {
            sprintf(sprintChar, "[EVSEOTA] bytes written %s", std::to_string(index + len).c_str());
            EVSELogger::info(sprintChar);
        }
    }

    if (final) {
        if (Update.end(true)) {
            EVSELogger::info("[EVSEOTA] Update successful");
        } else {
            sprintf(sprintChar, "[EVSEOTA] FAIL: %s", Update.errorString());
            EVSELogger::warn(sprintChar);
            Update.printError(Serial);
        }
    }
}