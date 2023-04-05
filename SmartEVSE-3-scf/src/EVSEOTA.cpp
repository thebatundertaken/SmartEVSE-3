#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Update.h>

#include "EVSELogger.h"
#include "EVSEOTA.h"

void EVSEOTA::updateGETRequestHandler(AsyncWebServerRequest* request) {
    request->send(200, "text/html",
                  "spiffs.bin updates the SPIFFS partition<br>firmware.bin updates the main firmware<br><form method='POST' action='/update' "
                  "enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>");
}

void EVSEOTA::updatePOSTRequestHandler(AsyncWebServerRequest* request) {
    bool shouldReboot = !Update.hasError();
    AsyncWebServerResponse* response = request->beginResponse(200, "text/plain", shouldReboot ? "OK" : "FAIL");
    response->addHeader("Connection", "close");
    request->send(response);
    delay(500);
    if (shouldReboot) {
        ESP.restart();
    }
}

void EVSEOTA::updateMultipartUploadHandler(AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
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