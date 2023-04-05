#/*
;    Project: Smart EVSE v3
;
;
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

#ifndef __EVSESCREEN
#define __EVSESCREEN

//#define BACKLIGHT_ON digitalWrite(PIN_LCD_LED, HIGH);
//#define BACKLIGHT_OFF digitalWrite(PIN_LCD_LED, LOW);

#include "EVSEController.h"

class EVSEScreen {
   public:
    EVSEScreen(){};

    void setup();
    void redraw();
    void lightUp();
    void resetLCD();
    void onUserActivity();
    unsigned long ScrollTimer = 0;
    uint8_t LCDpos = 0;

   private:
    void lightFadeOut();
    void drawMenu();

    uint8_t evseControllerOldState = STATE_A_STANDBY;
    uint8_t evseControllerOldErrorFlags = 0;
    uint8_t evseControllerOldMode = 0;
    uint8_t lcdBrightness = 0;
    unsigned long backlightTimer = 0;
};

extern EVSEScreen evseScreen;

#endif