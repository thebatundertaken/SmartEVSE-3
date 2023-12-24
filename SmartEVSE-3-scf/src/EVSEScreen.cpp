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

#include "EVSEScreen.h"

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>

#include "EVSEMenu.h"
#include "EVSEPin.h"
#include "glcd.h"
#include "utils.h"

#define LCD_BRIGHTNESS_MAX 255

// Seconds delay for the LCD backlight to turn off (fading out effect)
const uint16_t BACKLIGHT_ON_MILLIS = 15000;
const uint8_t FADE_OUT_STEP = 10;

void EVSEScreen::lightUp() {
    // Do not write command if LCD is at max bright, just reset timer
    if (lcdBrightness < LCD_BRIGHTNESS_MAX) {
        lcdBrightness = LCD_BRIGHTNESS_MAX;
        ledcWrite(LCD_CHANNEL, lcdBrightness);
    }

    backlightTimer = millis();
}

void EVSEScreen::lightFadeOut() {
    if (lcdBrightness > FADE_OUT_STEP) {
        lcdBrightness -= FADE_OUT_STEP;
        ledcWrite(LCD_CHANNEL, ease8InOutQuad(lcdBrightness));
    } else {
        lcdBrightness = 0;
        ledcWrite(LCD_CHANNEL, lcdBrightness);
    }
}

void EVSEScreen::resetLCD() {
    GLCD_init();
}

void EVSEScreen::onUserActivity() {
    lightUp();

    // reset timer for HelpMenu text
    ScrollTimerHelpMenu = millis();
    // reset position of scrolling text
    LCDpos = 0;

    // Fast LCD refresh and redraw to avoid 1sg default refresh & redraw
    redraw();
}

void EVSEScreen::setup() {
    ledcSetup(LCD_CHANNEL, 5000, 8);  // LCD channel 5, 5kHz, 8 bit
    ledcAttachPin(PIN_LCD_LED, LCD_CHANNEL);
    ledcWrite(LCD_CHANNEL, 0);

    lightUp();
    GLCD_init();
}

void EVSEScreen::drawMenu() {
    // Show / scroll menu help text
    if ((!evseMenu.subMenu) && ((millis() - ScrollTimerHelpMenu) >= DEFAULT_SCROLLTIMERHELPMENU_MILLIS)) {
        GLCDMenuItemHelp();
        return;
    }

    // TODO SCF fix shouldRedrawMenu
    // Prevent redraw the very same menu option again and again
    if (!evseMenu.shouldRedrawMenu()) {
        // return;
    }

    if (evseMenu.currentMenuOption == MENU_ENTER) {
        GLCDHoldForMenu();
        return;
    }

    GLCDMenu();
}

void EVSEScreen::redraw() {
    if (evseControllerOldState != evseController.state) {
        evseControllerOldState = evseController.state;
        lightUp();

        if (evseController.state == STATE_A_STANDBY || evseController.state == STATE_B_VEHICLE_DETECTED ||
            evseController.state == STATE_B1_VEHICLE_DETECTED_NO_POWER) {
            // Re-init LCD
            resetLCD();
        }
    }

    if (evseControllerOldErrorFlags != evseController.errorFlags) {
        evseControllerOldErrorFlags = evseController.errorFlags;
        lightUp();
    }

    if (evseControllerOldMode != evseController.mode) {
        evseControllerOldMode = evseController.mode;
        lightUp();
    }

    // Is LCD backlight on?
    if (backlightTimer > 0) {
        if ((millis() - backlightTimer) >= BACKLIGHT_ON_MILLIS) {
            backlightTimer = 0;
        }
    }

    if (backlightTimer == 0 && lcdBrightness > 0) {
        lightFadeOut();
    }

    if (evseMenu.currentMenuOption != MENU_NO_OPTION) {
        drawMenu();
        return;
    }

    if (evseController.errorFlags) {
        if (evseController.errorFlags & ERROR_FLAG_CT_NOCOMM) {
            GLCDControllerNoCTCommError();
            return;
        }

        if (evseController.errorFlags & ERROR_FLAG_TEMP_HIGH) {
            GLCDControllerTempHighError();
            return;
        }

        if (evseController.errorFlags & ERROR_FLAG_RCM_TRIPPED) {
            GLCDControllerRCMError();
            return;
        }

        if (evseController.errorFlags & ERROR_FLAG_BL_FLASH) {
            GLCDControllerOTAFlashError();
            return;
        }
    }

    if (evseController.mode == MODE_NORMAL || !evseRFID.isRFIDAccessGranted()) {
        GLCDNormalMode();
        return;
    }

    GLCDSmartSolarMode();
}

EVSEScreen evseScreen;