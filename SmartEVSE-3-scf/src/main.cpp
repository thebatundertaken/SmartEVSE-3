#include "main.h"

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>

#include "EVSEButtons.h"
#include "EVSEController.h"
#include "EVSELockActuator.h"
#include "EVSEMenu.h"
#include "EVSEModbus.h"
#include "EVSERFID.h"
#include "EVSERgbLeds.h"
#include "EVSEScreen.h"
#include "EVSEWifi.h"

void onTimer10ms(void* parameter) {
    while (1) {
        evseController.loop();
        if (evseLockActuator.isLockEnabled()) {
            evseLockActuator.loop();
        }
        // evseWifi.loop();
        // evseMenu.loop();
        evseButtons.loop();

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void onTimer100ms(void* parameter) {
    while (1) {
        evseModbus.loop();
        if (evseRFID.isEnabled()) {
            evseRFID.loop();
        }

        if(evseRgbLeds.ledsEnabled) {
            evseRgbLeds.loop();
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void createTasks() {
    xTaskCreate(onTimer10ms, "onTimer10ms", 10000, NULL, 5, NULL);
    xTaskCreate(onTimer100ms, "onTimer100ms", 10000, NULL, 4, NULL);
}

void setup() {
    evseController.setup();
    evseModbus.setup();
    evseRFID.setup();
    evseLockActuator.setup();
    evseWifi.setup();
    evseMenu.setup();
    evseButtons.setup();
    evseRgbLeds.setup();
    evseScreen.setup();

    createTasks();
}

void loop() {
    // Refresh LCD screen
    evseScreen.redraw();
    delay(1000);
}