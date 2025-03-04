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

#include "EVSEButtons.h"

#include "EVSEController.h"
#include "EVSEMenu.h"
#include "EVSEPin.h"
#include "EVSEScreen.h"

void EVSEButtons::setup() {
    // Nothing to do
}

// Sample the three < o > buttons
//         Value: 1 2 4
//         Bit: 0:Pressed / 1:Released
void EVSEButtons::sampleButtons() {
    buttonState = 0;

    // As the buttons are shared with the SPI lines going to the LCD,
    // we have to make sure that this does not interfere by write actions to the LCD.
    // Therefore updating the LCD is also done in this task.
    // disconnect MOSI pin
    pinMatrixOutDetach(PIN_LCD_SDO_B3, false, false);
    pinMode(PIN_LCD_SDO_B3, INPUT);
    pinMode(PIN_LCD_A0_B2, INPUT);
    // sample buttons < o >
    if (digitalRead(PIN_LCD_SDO_B3)) {
        buttonState = 4;  // > (right)
    }

    if (digitalRead(PIN_LCD_A0_B2)) {
        buttonState |= 2;  // o (middle)
    }

    if (digitalRead(PIN_IO0_B1)) {
        buttonState |= 1;  // < (left)
    }

    pinMode(PIN_LCD_SDO_B3, OUTPUT);
    // re-attach MOSI pin
    pinMatrixOutAttach(PIN_LCD_SDO_B3, VSPID_IN_IDX, false, false);
    pinMode(PIN_LCD_A0_B2, OUTPUT);
}

void EVSEButtons::loop() {
    sampleButtons();

    // No button pressed nor long pressed
    if ((buttonState == BUTTON_NONE_MASK) && (buttonState == oldButtonState)) {
        return;
    }

    oldButtonState = buttonState;

#if EVSE_FEATFLAG_ENABLE_RCMON
    // Clear RCM error by pressing any button
    evseController.resetRCMErrorFlag();
#endif
    evseMenu.onButtonChanged(buttonState);
    evseScreen.onUserActivity();
}

EVSEButtons evseButtons;