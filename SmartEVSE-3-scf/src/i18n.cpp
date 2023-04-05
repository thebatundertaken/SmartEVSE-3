#include <stdio.h>
#include <stdlib.h>
#include "i18n.h"

const char* geti18nStrMeterText(uint8_t statusCode) {
    if (statusCode < 9) {
        return i18nStrMeter[statusCode];
    }

    return "Unknown";
}

const char* geti18nRfidStatusText(uint8_t code) {
    if (code < 8) {
        return i18nStrRFIDStatus[code];
    }

    return "Unknown";
}

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
