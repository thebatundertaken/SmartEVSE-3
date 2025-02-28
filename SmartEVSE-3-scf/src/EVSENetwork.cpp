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

#include "EVSENetwork.h"

#include <ArduinoJson.h>

#include "EVSECluster.h"
#include "EVSEController.h"
#include "EVSELogger.h"
#include "EVSEModbus.h"
#include "EVSERFID.h"
#include "EVSEWifi.h"
#include "i18n.h"
#include "main.h"

void EVSENetwork::getWebUIData(AsyncWebServerRequest* request) {
    DynamicJsonDocument doc(1200);
    doc["version"] = String(EVSE_VERSION);
    doc["hostname"] = String(evseWifi.getApHostname());

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
    doc["cluster"]["loadbl"] = evseCluster.getLoadBl();

    doc["modbus"]["mainsMeterText"] = geti18nStrMeterText(evseModbus.mainsMeter);
    doc["modbus"]["evMeterText"] = geti18nStrMeterText(evseModbus.evMeter);
    doc["modbus"]["pvMeterText"] = geti18nStrMeterText(evseModbus.pvMeter);
    doc["modbus"]["lastMainsMeterResponse"] = evseController.getLastMainsMeterResponse();
    doc["modbus"]["powerMeasured"] = evseModbus.powerMeasured;
    doc["modbus"]["grid"] = evseModbus.grid;
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

void EVSENetwork::postSettings(AsyncWebServerRequest* request) {
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

void EVSENetwork::postSensorboxReadings(AsyncWebServerRequest* request) {
    if (!request->hasParam("L1") || !request->hasParam("L2") || !request->hasParam("L3")) {
        request->send(200, "text/plain", "Missing readings");
        return;
    }

    evseController.mainsMeterReadings(request->getParam("L1")->value().toInt(),
                                      request->getParam("L2")->value().toInt(),
                                      request->getParam("L3")->value().toInt());

    request->send(200, "text/plain", "OK");
}

void EVSENetwork::postReboot(AsyncWebServerRequest* request) {
    ESP.restart();

    DynamicJsonDocument doc(200);
    doc["reboot"] = true;

    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
}

void EVSENetwork::forceDisconnect(AsyncWebServerRequest* request) {
    DynamicJsonDocument doc(200);
    evseController.forceDisconnect();
    doc["forceDisconnect"] = true;

    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
}

void EVSENetwork::forceCharge(AsyncWebServerRequest* request) {
    DynamicJsonDocument doc(200);
    doc["forceCharge"] = evseController.forceStartCharging();

    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
}
