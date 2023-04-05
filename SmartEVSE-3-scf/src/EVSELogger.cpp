#include <Arduino.h>

#include "EVSELogger.h"

void EVSELogger::error(const String &s) {
    Serial.print(s);
    Serial.print("\n");
}

void EVSELogger::warn(const String &s) {
    Serial.print(s);
    Serial.print("\n");
}

void EVSELogger::info(const String &s) {
    Serial.print(s);
    Serial.print("\n");
}

void EVSELogger::debug(const String &s) {
    Serial.print(s);
    Serial.print("\n");
}

