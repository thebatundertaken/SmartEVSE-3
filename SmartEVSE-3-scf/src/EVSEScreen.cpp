#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>

#include "EVSEMenu.h"
#include "EVSEPin.h"
#include "EVSEScreen.h"
#include "glcd.h"
#include "utils.h"

#define LCD_BRIGHTNESS_MAX 255

// Seconds delay for the LCD backlight to turn off (fading out effect)
const uint16_t BACKLIGHT_ON_MILLIS = 30000;
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
    ScrollTimer = millis();
    // reset position of scrolling text
    LCDpos = 0;
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
    if ((!evseMenu.subMenu) && (ScrollTimer + 5000 < millis())) {
        GLCDMenuItemHelp();
        return;
    }

    // Prevent redraw the very same menu option again and again
    if (!evseMenu.shouldRedrawMenu()) {
        return;
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
    }

    GLCDSmartSolarMode();
}

EVSEScreen evseScreen;