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

#include "main.h"

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>

#include "EVSEButtons.h"
#include "EVSECluster.h"
#include "EVSEController.h"
#include "EVSELockActuator.h"
#include "EVSEMenu.h"
#include "EVSEModbus.h"
#include "EVSERFID.h"
#include "EVSERgbLeds.h"
#include "EVSEScreen.h"
#include "EVSEWifi.h"

uint8_t screenRedraw = 1;

void onTimer15ms(void* parameter) {
    while (1) {
        evseController.loop();
        vTaskDelay(15 / portTICK_PERIOD_MS);
    }
}

void onTimer100ms(void* parameter) {
    while (1) {
        // As the buttons are shared with the SPI lines going to the LCD,
        // we have to make sure that this does not interfere by write actions to the LCD
        // Therefore updating the LCD is also done in this task
        evseButtons.loop();

        // Redraw LCD screen every 1 sec
        if (--screenRedraw == 0) {
            evseScreen.redraw();
            screenRedraw = 10;
        }

        evseModbus.loop();
        if (evseLockActuator.isLockEnabled()) {
            evseLockActuator.loop();
        }

        if (evseRFID.isEnabled()) {
            evseRFID.loop();
        }

        if (evseRgbLeds.ledsEnabled) {
            evseRgbLeds.loop();
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void createTasks() {
    xTaskCreate(onTimer15ms, "onTimer15ms", 10000, NULL, 15, NULL);
    xTaskCreate(onTimer100ms, "onTimer100ms", 10000, NULL, 10, NULL);
}

void setup() {
    evseController.setup();
    evseModbus.setup();
    evseCluster.setup();
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
    delay(1000);
}