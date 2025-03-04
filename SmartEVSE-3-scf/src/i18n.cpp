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

#include "i18n.h"

#include <stdio.h>
#include <stdlib.h>

const char* geti18nStrMeterText(uint8_t statusCode) {
    if (statusCode < 10) {
        return i18nStrMeter[statusCode];
    }

    return "Unknown";
}

#if EVSE_FEATFLAG_ENABLE_RFID
const char* geti18nRfidStatusText(uint8_t code) {
    if (code < 8) {
        return i18nStrRFIDStatus[code];
    }

    return "Unknown";
}
#endif

const char* geti18nStateText(uint8_t stateCode) {
    if (stateCode < 11) {
        return i18nStrStateNameWeb[stateCode];
    }

    return "Unknown";
}

const char* geti18nErrorText(uint8_t ErrorCode) {
    uint8_t count = 0;
    // find the error bit that is set
    while (ErrorCode) {
        count++;
        ErrorCode = ErrorCode >> 1;
    }
    return (count < 9) ? i18nStrErrorNameWeb[count] : "Multiple Errors";
}
