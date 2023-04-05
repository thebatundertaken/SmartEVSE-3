#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>

#include "EVSEButtons.h"
#include "EVSEPin.h"
#include "EVSEMenu.h"
#include "EVSEController.h"
#include "EVSEScreen.h"

void EVSEButtons::setup() {
    // < button
    pinMode(PIN_IO0_B1, INPUT);
    // o Select button + A0 LCD
    pinMode(PIN_LCD_A0_B2, OUTPUT);
    // > button + SDA/MOSI pin
    pinMode(PIN_LCD_SDO_B3, OUTPUT);
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

    evseScreen.onUserActivity();

    //Clear RCM error by pressing any button
    evseController.resetRCMErrorFlag();

    evseMenu.onButtonChanged(buttonState);

    oldButtonState = buttonState;

    evseScreen.redraw();
}

EVSEButtons evseButtons;