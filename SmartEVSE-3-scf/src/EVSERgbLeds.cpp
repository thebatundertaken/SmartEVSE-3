#include <Arduino.h>
#include <Preferences.h>

#include "EVSEController.h"
#include "EVSELogger.h"
#include "EVSEPin.h"
#include "EVSERFID.h"
#include "EVSERgbLeds.h"
#include "utils.h"

// RGB leds
#define STATE_A_LED_BRIGHTNESS 40
#define STATE_B_LED_BRIGHTNESS 255
#define ERROR_LED_BRIGHTNESS 255
#define WAITING_LED_BRIGHTNESS 255

const char* PREFS_LEDS_NAMESPACE = "settings";
const char* PREFS_LEDSENABLED_KEY = "Leds";

void EVSERgbLeds::adjustLedsByPwm() {
    // Orange/Yellow for Solar mode, green for Normal/Smart mode
    if (evseController.mode == MODE_SOLAR) {
        color.R = ledPwm;
        color.G = ledPwm * 2 / 3;
    } else {
        color.R = 0;
        color.G = ledPwm;
    }
    color.B = 0;
}

void EVSERgbLeds::adjustLeds() {
    if (evseController.errorFlags & (ERROR_FLAG_RCM_TRIPPED | ERROR_FLAG_CT_NOCOMM)) {
        // Very rapid flashing, RCD tripped or no Serial Communication
        ledCount += 20;
        // Red LED 50% of time on, full brightness
        color.R = (ledCount > 128) ? ERROR_LED_BRIGHTNESS : 0;
        color.G = 0;
        color.B = 0;
        return;
    }

    // Waiting for Solar power or not enough current to start charging. Slow blinking.
    if (evseController.errorFlags || evseController.isChargeDelayed()) {
        ledCount += 2;
        // LED 10% of time on, full brightness
        ledPwm = (ledCount > 230) ? WAITING_LED_BRIGHTNESS : 0;
        adjustLedsByPwm();
        return;
    }

    // No Access, LEDs off
    if (evseRFID.RFIDReader != RFID_READER_DISABLED && evseRFID.rfidAccessBit == RFID_NO_ACCESS) {
        color.R = 0;
        color.G = 0;
        color.B = 0;
        ledPwm = 0;
        return;
    }

    // State A, B or C
    switch (evseController.state) {
        case STATE_A_STANDBY:
            // STATE A, LED on (dimmed)
            ledPwm = STATE_A_LED_BRIGHTNESS;
            break;

        case STATE_B_VEHICLE_DETECTED:
        case STATE_B1_VEHICLE_DETECTED_NO_POWER:
            // STATE B, LED on (full brightness)
            ledPwm = STATE_B_LED_BRIGHTNESS;
            // When switching to STATE C, start at full brightness
            ledCount = 128;
            break;

        case STATE_C_CHARGING:
            if (evseController.mode == MODE_SOLAR) {
                // Slower fading (Solar mode)
                ledCount++;
            } else {
                // Faster fading (Smart mode)
                ledCount += 2;
            }
            // pre calculate new ledPwm value
            ledPwm = ease8InOutQuad(triwave8(ledCount));
            break;
    }

    adjustLedsByPwm();
}

void EVSERgbLeds::readEpromSettings() {
    Preferences preferences;
    if (preferences.begin(PREFS_LEDS_NAMESPACE, true) != true) {
        EVSELogger::error("Unable to open preferences for EVSERgbLeds");
        return;
    }

    bool firstRun = true;
    if (preferences.isKey(PREFS_LEDSENABLED_KEY)) {
        firstRun = false;

        ledsEnabled = preferences.getUChar(PREFS_LEDSENABLED_KEY, DEFAULT_LEDS_ENABLED);
    }
    preferences.end();

    if (firstRun) {
        ledsEnabled = DEFAULT_LEDS_ENABLED;
        writeEpromSettings();
    }
}

void EVSERgbLeds::writeEpromSettings() {
    Preferences preferences;
    if (preferences.begin(PREFS_LEDS_NAMESPACE, false) != true) {
        EVSELogger::error("Unable to write preferences for EVSERgbLeds");
        return;
    }

    preferences.putUChar(PREFS_LEDSENABLED_KEY, ledsEnabled);

    preferences.end();
}

void EVSERgbLeds::setup() {
    readEpromSettings();
};

void EVSERgbLeds::loop() {
    adjustLeds();

    if (ledsEnabled) {
        ledcWrite(RED_CHANNEL, color.R);
        ledcWrite(GREEN_CHANNEL, color.G);
        ledcWrite(BLUE_CHANNEL, color.B);
    }
}

EVSERgbLeds evseRgbLeds;