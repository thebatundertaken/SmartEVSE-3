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

#ifndef __EVSELOCKACTUATOR
#define __EVSELOCKACTUATOR

#include <Arduino.h>

// No Cable lock
#define LOCK_DISABLED 0
// Solenoid
#define LOCK_SOLENOID 1
// Motor
#define LOCK_MOTOR 2

#define STATUS_UNLOCKED 0
#define STATUS_LOCKED 1

class EVSELockActuator {
   public:
    EVSELockActuator() {};

    void setup();
    void loop();
    void updateSettings();
    void resetSettings();

    // Cable lock (0:Disable / 1:Solenoid / 2:Motor)
    uint8_t getLockType() { return lockType; }
    void setLockType(uint8_t lockType);
    bool isLockEnabled() { return lockType != LOCK_DISABLED; };

   protected:
    static void onCableLockUnlockThread(EVSELockActuator* myself);

   private:
    void readEpromSettings();
    void writeEpromSettings();

    void setUnlockCable(bool val);
    void setLockCable(bool val);
    bool checkPinCableLocked();
    bool checkPinCableUnlocked();

    void startCableLockUnlockThread();
    void stopCableLockUnlockThread();
    void unlockCableWorkflow();
    void lockCableWorkflow();

    TaskHandle_t taskHandle = NULL;

    unsigned long cableUnlockMillis = 0;
    unsigned long cableLockMillis = 0;
    bool unlockCable = false;
    bool lockCable = false;

    // Cable lock (0:Disable / 1:Solenoid / 2:Motor)
    uint8_t lockType = LOCK_DISABLED;
    uint8_t lock1 = 0;
    uint8_t lock2 = 1;

    uint8_t status = STATUS_UNLOCKED;
};

extern EVSELockActuator evseLockActuator;

#endif