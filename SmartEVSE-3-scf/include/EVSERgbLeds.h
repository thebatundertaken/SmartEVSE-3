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

#ifndef __EVSERGBLEDS
#define __EVSERGBLEDS

#define DEFAULT_LEDS_ENABLED 1

struct RgbColor {
    uint8_t R = 0;
    uint8_t G = 0;
    uint8_t B = 0;
};

class EVSERgbLeds {
   public:
    EVSERgbLeds(){};

    void setup();
    void loop();
    RgbColor getRgbColor() { return color; };
    bool ledsEnabled = DEFAULT_LEDS_ENABLED;

   private:
    void adjustLeds();
    void adjustLedsByPwm();
    void readEpromSettings();
    void writeEpromSettings();

    // RGB leds
    RgbColor color;
    // Raw Counter before being converted to PWM value
    uint8_t ledCount = 0;
    // PWM value 0-255
    unsigned int ledPwm = 0;
};

extern EVSERgbLeds evseRgbLeds;

#endif