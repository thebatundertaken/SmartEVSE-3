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

void EVSEOTA::updateGETRequestHandler(AsyncWebServerRequest* request) {
  request->send(200, "text/html",
                "spiffs.bin updates the SPIFFS partition<br>firmware.bin updates the main "
                "firmware<br><form method='POST' action='/update' "
                "enctype='multipart/form-data'><input type='file' name='update'><input "
                "type='submit' value='Update'></form>");
}

void EVSEOTA::updatePOSTRequestHandler(AsyncWebServerRequest* request) {
  bool shouldReboot = !Update.hasError();
  AsyncWebServerResponse* response =
      request->beginResponse(200, "text/plain", shouldReboot ? "OK" : "FAIL");
  response->addHeader("Connection", "close");
  request->send(response);
  delay(500);
  if (shouldReboot) {
    ESP.restart();
  }
}

void EVSEOTA::updateMultipartUploadHandler(AsyncWebServerRequest* request, String filename,
                                           size_t index, uint8_t* data, size_t len, bool final) {
  if (!index) {
    EVSELogger::info("\nUpdate Start:");
    EVSELogger::info(filename);
    if (filename == "spiffs.bin") {
      EVSELogger::info("\nSPIFFS partition write");
      if (!Update.begin(SPI_PARTITION_SIZE, U_SPIFFS)) {
        Update.printError(Serial);
      }
    } else if (!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000), U_FLASH) {
      Update.printError(Serial);
    }
  }

  if (!Update.hasError()) {
    if (Update.write(data, len) != len) {
      Update.printError(Serial);
    } else {
      EVSELogger::info("bytes written");
      EVSELogger::info(std::to_string(index + len).c_str());
    }
  }

  if (final) {
    if (Update.end(true)) {
      EVSELogger::info("Update successful");
    } else {
      Update.printError(Serial);
    }
  }
}