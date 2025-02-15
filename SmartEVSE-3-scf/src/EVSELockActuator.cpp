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

#include "EVSELockActuator.h"

#include <Arduino.h>
#include <Preferences.h>

#include "EVSEController.h"
#include "EVSELogger.h"
#include "EVSEPin.h"
#include "EVSERFID.h"

#define ACTUATOR_LOCK                       \
    {                                       \
        digitalWrite(PIN_ACTUATOR_B, HIGH); \
        digitalWrite(PIN_ACTUATOR_A, LOW);  \
    }
#define ACTUATOR_UNLOCK                     \
    {                                       \
        digitalWrite(PIN_ACTUATOR_B, LOW);  \
        digitalWrite(PIN_ACTUATOR_A, HIGH); \
    }

#define ACTUATOR_OFF                        \
    {                                       \
        digitalWrite(PIN_ACTUATOR_B, HIGH); \
        digitalWrite(PIN_ACTUATOR_A, HIGH); \
    }

const char* PREFS_LOCKACTUATOR_NAMESPACE = "settings";
const char* PREFS_LOCK_KEY = "Lock";

bool EVSELockActuator::checkPinCableLocked() {
    return digitalRead(PIN_LOCK_IN) == lock1;
}

bool EVSELockActuator::checkPinCableUnlocked() {
    return digitalRead(PIN_LOCK_IN) == lock2;
}

void EVSELockActuator::setLockType(uint8_t newLockType) {
    switch (newLockType) {
        case LOCK_SOLENOID:
            lockType = newLockType;
            lock1 = LOW;
            lock2 = HIGH;
            break;

        case LOCK_MOTOR:
            lockType = newLockType;
            lock1 = HIGH;
            lock2 = LOW;
            break;

        default:
            lockType = LOCK_DISABLED;
    }
}

void EVSELockActuator::readEpromSettings() {
    Preferences preferences;
    if (preferences.begin(PREFS_LOCKACTUATOR_NAMESPACE, true) != true) {
        EVSELogger::error("Unable to open preferences for EVSELockActuator");
        return;
    }

    bool firstRun = true;
    if (preferences.isKey(PREFS_LOCK_KEY)) {
        firstRun = false;

        setLockType(preferences.getUChar(PREFS_LOCK_KEY, LOCK_DISABLED));
    }
    preferences.end();

    if (firstRun) {
        lockType = LOCK_DISABLED;
        writeEpromSettings();
    }
}

void EVSELockActuator::writeEpromSettings() {
    Preferences preferences;
    if (preferences.begin(PREFS_LOCKACTUATOR_NAMESPACE, false) != true) {
        EVSELogger::error("Unable to write preferences for EVSELockActuator");
        return;
    }

    preferences.putUChar(PREFS_LOCK_KEY, lockType);

    preferences.end();
}

void EVSELockActuator::updateSettings() {
    writeEpromSettings();
}

void EVSELockActuator::resetSettings() {
    readEpromSettings();
}

void EVSELockActuator::unlockCableWorkflow() {
    unsigned long elapsed = millis() - cableUnlockMillis;

    // 600ms pulse
    if (elapsed < 600) {
        ACTUATOR_UNLOCK;
        return;
    }

    // 100ms pulse
    if (elapsed < 700) {
        ACTUATOR_OFF;
        return;
    }

    // If it is not lock, we're done
    if (!checkPinCableLocked()) {
        // Done
        unlockCable = false;
        return;
    }

    // still locked? try to unlock again in 5 seconds
    if (elapsed > 5000) {
        cableUnlockMillis = millis();
    }
}

void EVSELockActuator::lockCableWorkflow() {
    unsigned long elapsed = millis() - cableLockMillis;

    // 600ms pulse
    if (elapsed < 600) {
        ACTUATOR_LOCK;
        return;
    }

    // 100ms pulse
    if (elapsed < 700) {
        ACTUATOR_OFF;
        return;
    }

    // If it is not unlock, we're done
    if (!checkPinCableUnlocked()) {
        // Done
        lockCable = false;
        return;
    }

    // still unlocked? try to unlock again in 5 seconds
    if (elapsed > 5000) {
        cableLockMillis = millis();
    }
}

void EVSELockActuator::onCableLockUnlockThread(EVSELockActuator* myself) {
    while (myself->isLockEnabled()) {
        while (myself->lockCable || myself->unlockCable) {
            // Unlock cable takes precedence over LockCable
            if (myself->unlockCable) {
                myself->unlockCableWorkflow();

                // Reset lock millis due to precedence
                if (myself->lockCable) {
                    myself->cableLockMillis = millis();
                }
                continue;
            }

            // Lock Cable
            if (myself->lockCable) {
                myself->lockCableWorkflow();
                continue;
            }

            // Pause the task for 100ms
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }

    myself->stopCableLockUnlockThread();
}

void EVSELockActuator::startCableLockUnlockThread() {
    if (isLockEnabled() && taskHandle == NULL) {
        xTaskCreate((TaskFunction_t)&onCableLockUnlockThread, "onCableLock", 4096, &evseLockActuator, 5, &taskHandle);
    }
}

void EVSELockActuator::stopCableLockUnlockThread() {
    if (taskHandle != NULL) {
        vTaskDelete(taskHandle);
        taskHandle = NULL;
    }
}

void EVSELockActuator::setUnlockCable(bool val) {
    unlockCable = val;
    cableUnlockMillis = millis();
    startCableLockUnlockThread();
}

void EVSELockActuator::setLockCable(bool val) {
    lockCable = val;
    cableLockMillis = millis();
    startCableLockUnlockThread();
}

void EVSELockActuator::loop() {
    if (!isLockEnabled()) {
        return;
    }

    //  One RFID card can lock/unlock the charging socket (like a public charging station)
    if (evseRFID.RFIDReader == RFID_READER_ENABLE_ONE) {
        setUnlockCable(evseRFID.rfidAccessBit == RFID_NO_ACCESS);
    } else {
        // The charging socket is unlocked when charging stops.
        setUnlockCable(evseController.state != STATE_C_CHARGING);
    }

    // If the cable is connected to the EV, the cable will be locked.
    setLockCable(evseController.state != STATE_A_STANDBY);
}

void EVSELockActuator::setup() {
    readEpromSettings();
}

EVSELockActuator evseLockActuator;