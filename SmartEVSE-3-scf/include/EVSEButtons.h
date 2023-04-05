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

#ifndef __EVSEBUTTONS
#define __EVSEBUTTONS

#define BUTTON_O_MASK 0x5
#define BUTTON_LEFT_MASK 0x6
#define BUTTON_RIGHT_MASK 0x3
#define BUTTON_LEFT_AND_RIGHT_MASK 0x2
#define BUTTON_NONE_MASK 0x7

class EVSEButtons {
   public:
    EVSEButtons(){};

    void setup();
    void loop();

   private:
    void sampleButtons();

    // Holds latest push Buttons state (LSB 3:0)
    uint8_t buttonState = 0x0f;
    // Holds previous push Buttons state (LSB 3:0)
    uint8_t oldButtonState = 0x0f;
};

extern EVSEButtons evseButtons;

#endif