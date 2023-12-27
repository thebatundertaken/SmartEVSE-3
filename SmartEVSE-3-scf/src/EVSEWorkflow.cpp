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

#include "EVSEWorkflow.h"

#include <Arduino.h>

#include "EVSECluster.h"
#include "EVSEController.h"
#include "EVSELogger.h"
#include "EVSEPin.h"
#include "EVSERFID.h"

void EVSEWorkflow::workflowStandby() {
    switch (evseController.getControlPilot()) {
        case CONTROL_PILOT_12V:
            // Disconnected or no power
            if (evseController.state != STATE_A_STANDBY) {
                // reset state incase we were stuck in STATE_MODBUS_COMM_B
                evseController.setState(STATE_A_STANDBY);
                evseController.onStandbyReadyToCharge();
            }

            break;

        case CONTROL_PILOT_9V:
            // switch to State B ?
            if (evseController.errorFlags || evseController.isChargeDelayed() || !evseRFID.isRFIDAccessGranted() ||
                evseController.state == STATE_MODBUS_COMM_B) {
                break;
            }

            evseController.onVehicleConnected();

            // Node wants to switch to State B
            if (evseCluster.amIWorkerNode()) {
                evseController.setState(STATE_MODBUS_COMM_B);
                break;
            }

            if (!evseCluster.isEnoughCurrentAvailableForAnotherEVSE()) {
                EVSELogger::warn("[EVSEWorkflow] Vehicle just connected but not enough power");
                evseController.onNotEnoughPower();
            }

            evseController.setState(STATE_B_VEHICLE_DETECTED);
            break;
    }
}

void EVSEWorkflow::checkCurrentAvailableWaitMillis() {
    if ((currentAvailableWaitMillis == 0) ||
        ((millis() - currentAvailableWaitMillis) < CURRENTAVAILABLE_WAIT_TIMEOUT_MILLIS)) {
        return;
    }

    //  Current NOT in use after 30 seconds, disconnect
    currentAvailableWaitMillis = 0;

    evseController.setState(STATE_DISCONNECT_IN_PROGRESS);
    evseController.onDisconnectInProgress();
}

void EVSEWorkflow::workflowVehicleDetected() {
    switch (evseController.getControlPilot()) {
        case CONTROL_PILOT_12V:
            // Disconnected or no PWM
            evseController.setState(STATE_A_STANDBY);
            break;

        case CONTROL_PILOT_DIODE_CHECK_OK:
            EVSELogger::info("[EVSEWorkflow] Diode found, OK");
            isDiodeOk = true;
            evseController.onDiodeCheckOK();
            break;

        case CONTROL_PILOT_6V:
            // Vehicle wants to charge
            if (!isDiodeOk || evseController.errorFlags || evseController.isChargeDelayed()) {
                break;
            }

            if (evseCluster.amIWorkerNode()) {
                if (evseController.state != STATE_MODBUS_COMM_C) {
                    EVSELogger::info("[EVSEWorkflow] Worker node vehicle starting to charge");
                    evseController.setState(STATE_MODBUS_COMM_C);
                }
                break;
            }

            EVSELogger::info("[EVSEWorkflow] Vehicle starting to charge");
            // Vehicle is charging, disable timeout
            currentAvailableWaitMillis = 0;

            // Adjust chargeCurrent BEFORE opening electric contactor
            evseController.onVehicleStartCharging();
            prevChargeCurrent = evseController.getChargeCurrent();

            evseController.setState(STATE_C_CHARGING);
            break;
    }

    checkCurrentAvailableWaitMillis();
}

void EVSEWorkflow::workflowCharging() {
    switch (evseController.getControlPilot()) {
        case CONTROL_PILOT_6V:
            // Vehicle is consuming power. Checking it's within specs
            chargeCurrent = evseController.getChargeCurrent();
            if (chargeCurrent != prevChargeCurrent) {
                sprintf(sprintfStr, "[EVSEWorkflow] [C] Detected chargeCurrent rebalance from %u to %u",
                        prevChargeCurrent, chargeCurrent);
                EVSELogger::debug(sprintfStr);

                prevChargeCurrent = chargeCurrent;
            }

            break;

        case CONTROL_PILOT_12V:
            // Disconnected or no PWM
            EVSELogger::info("[EVSEWorkflow] [C] Vehicle disconnected or no PWM");
            evseController.setState(STATE_A_STANDBY);
            break;

        case CONTROL_PILOT_9V:
            // Connected but not requesting power (maybe Tesla app off-peak charge,
            // battery at 100%, battery charge limit)
            EVSELogger::info("[EVSEWorkflow] [C] Connected vehicle stopped charging");
            evseController.setState(STATE_DISCONNECT_IN_PROGRESS);
            break;
    }
}

void EVSEWorkflow::workflowChargingNoPower() {
    switch (evseController.getControlPilot()) {
        case CONTROL_PILOT_12V:
            // Disconnected or no PWM
            EVSELogger::info("[EVSEWorkflow] [C1] Vehicle disconnected or no PWM");
            chargingNoPwmTimeoutMillis = 0;
            evseController.setState(STATE_A_STANDBY);
            break;

        case CONTROL_PILOT_9V:
            EVSELogger::info("[EVSEWorkflow] [C1] Vehicle detected no power");
            chargingNoPwmTimeoutMillis = 0;
            evseController.setState(STATE_B1_VEHICLE_DETECTED_NO_POWER);
            break;

        default:
            // if the EV does not stop charging in 6 seconds, we will open the
            // contactor anyway
            if ((chargingNoPwmTimeoutMillis > 0) &&
                ((millis() - chargingNoPwmTimeoutMillis) >= CHARGING_NO_POWER_WAIT_TIMEOUT_MILLIS)) {
                chargingNoPwmTimeoutMillis = 0;
                EVSELogger::info("[EVSEWorkflow] [C1] Charging no power timeout");
                evseController.setState(STATE_B1_VEHICLE_DETECTED_NO_POWER);
            }
    }
}

void EVSEWorkflow::workflowDisconnecting() {
    // 3 seconds wait before disconnecting to let car process on-the-fly electric signals
    if (disconnectWaitMillis == 0) {
        disconnectWaitMillis = millis();
        return;
    }

    if ((millis() - disconnectWaitMillis) < DISCONNECT_WAIT_MILLIS) {
        return;
    }

    EVSELogger::info("[EVSEWorkflow] Gracefully disconected");
    disconnectWaitMillis = 0;
    evseController.setState(STATE_B_VEHICLE_DETECTED);
}

void EVSEWorkflow::loop() {
    stateChanged = prevstate != evseController.state;
    if (stateChanged) {
        // On transition, reset isDiodeOk
        isDiodeOk = false;

        sprintf(sprintfStr, "[EVSEWorkflow] Transitioning to state (%c) [code %u]", (evseController.state + 65),
                evseController.state);
        EVSELogger::info(sprintfStr);
    }

    switch (evseController.state) {
        case STATE_A_STANDBY:
        case STATE_B1_VEHICLE_DETECTED_NO_POWER:
        case STATE_MODBUS_COMM_B:
            if (stateChanged) {
                EVSELogger::info("[EVSEWorkflow] [A/B1/MBC] Stand by");

                currentAvailableWaitMillis = 0;
            }

            workflowStandby();
            break;

        case STATE_B_VEHICLE_DETECTED:
        case STATE_MODBUS_COMM_C:
            if (stateChanged) {
                EVSELogger::info("[EVSEWorkflow] [B/MBC] Vehicle detected");

                // Turn off current if unused for 30 seconds
                currentAvailableWaitMillis = millis();
            }

            workflowVehicleDetected();
            break;

        case STATE_C_CHARGING:
            if (stateChanged) {
                sprintf(sprintfStr, "[EVSEWorkflow] [C] Charging at %u (min %u; max %u)",
                        evseController.getChargeCurrent(), evseController.minEVCurrent * 10,
                        evseController.maxDeviceCurrent * 10);
                EVSELogger::info(sprintfStr);

                // Vehicle is charging, disable currentAvailableWait timeout
                currentAvailableWaitMillis = 0;
            }

            workflowCharging();
            break;

        case STATE_C1_CHARGING_NO_POWER:
            if (stateChanged) {
                EVSELogger::info("[EVSEWorkflow] [C1] Charging no power");

                // Wait maximum 6 seconds (CHARGING_NO_POWER_WAIT_TIMEOUT_MILLIS),
                // before forcing the contactor off
                chargingNoPwmTimeoutMillis = millis();
            }

            workflowChargingNoPower();
            break;

        case STATE_DISCONNECT_IN_PROGRESS:
            if (stateChanged) {
                EVSELogger::info("[EVSEWorkflow] [D] Disconnect in progress");
            }

            workflowDisconnecting();
            break;

        case STATE_MODBUS_COMM_B_OK:
            evseController.setState(STATE_B_VEHICLE_DETECTED);
            break;

        case STATE_MODBUS_COMM_C_OK:
            evseController.setState(STATE_C_CHARGING);
            break;
    }

    prevstate = evseController.state;
}

EVSEWorkflow evseWorkflow;