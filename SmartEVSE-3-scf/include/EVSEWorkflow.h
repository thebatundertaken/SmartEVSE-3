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

#ifndef __EVSEWORKFLOW
#define __EVSEWORKFLOW

#include <Arduino.h>

#include "EVSEController.h"

// if the EV does not stop charging in 6 seconds, we will open the contactor.
#define CHARGING_NO_POWER_WAIT_TIMEOUT_MILLIS 6000
// Turn off current if unused for 30 seconds
#define CURRENTAVAILABLE_WAIT_TIMEOUT_MILLIS 30000
// Let car 3 seconds to process electric signal on the fly before disconnecting
#define DISCONNECT_WAIT_MILLIS 3000

class EVSEWorkflow {
   public:
    EVSEWorkflow(){};

    void loop();

   private:
    uint8_t controllerState;
    uint8_t prevstate = STATE_A_STANDBY;
    uint16_t chargeCurrent = 0;
    uint16_t prevChargeCurrent = 0;
    bool isDiodeOk = false;
    bool stateChanged = false;
    char sprintfStr[128];
    // Graceful disconnect
    unsigned long disconnectWaitMillis = 0;
    // Turn off current if unused for CURRENTAVAILABLE_WAIT_TIMEOUT_MILLIS
    unsigned long currentAvailableWaitMillis = 0;
    unsigned long chargingNoPwmTimeoutMillis = 0;

    void workflowStandby();
    void workflowVehicleDetected();
    void workflowCharging();
    void workflowChargingNoPower();
    void workflowDisconnecting();
    void checkCurrentAvailableWaitMillis();
};

extern EVSEWorkflow evseWorkflow;

#endif