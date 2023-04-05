#include <Preferences.h>

#include "EVSEController.h"
#include "EVSELogger.h"
#include "EVSERFID.h"
#include "OneWire.h"

const char* PREFS_RFID_NAMESPACE = "settings";
const char* PREFS_RFIDREADER_KEY = "RFIDReader";

void EVSERFID::disableAccessTimeout() {
    if (RFIDReader != RFID_READER_ENABLE_ONE) {
        return;
    }

    // After sucessfully granted access by RFID card, that access will be revoked after 60 seconds
    //  Car must start charging within those 60 seconds, otherwise deactivate RFID access
    if (rfidLockWaitMillis == 0) {
        if (isRFIDAccessGranted()) {
            rfidLockWaitMillis = millis();
        }
        return;
    }

    if (evseController.state != STATE_A_STANDBY) {
        // Not in standby state, then disable timer
        rfidLockWaitMillis = 0;
        return;
    }

    if ((millis() - rfidLockWaitMillis) >= RFID_REVOKE_ACCESS_TIMEOUT_MILLIS) {
        rfidLockWaitMillis = 0;
        rfidAccessBit = RFID_NO_ACCESS;
    }
};

void EVSERFID::setRFIDReader(uint8_t value) {
    RFIDReader = value;

    switch (RFIDReader) {
        case RFID_READER_ENABLE_ONE:
            // RFID reader set to Enable One card, the EVSE is disabled by default
            rfidAccessBit = RFID_NO_ACCESS;
            break;

        case RFID_READER_DELETE_ALL:
            // Erase all RFID cards from ram + eeprom if set to EraseAll
            DeleteAllRFID();
            break;
    }
}

void EVSERFID::readEpromSettings() {
    Preferences preferences;
    if (preferences.begin(PREFS_RFID_NAMESPACE, true) != true) {
        EVSELogger::error("Unable to open preferences for EVSEModbus");
        return;
    }

    bool firstRun = true;
    if (preferences.isKey(PREFS_RFIDREADER_KEY)) {
        firstRun = false;

        setRFIDReader(preferences.getUChar(PREFS_RFIDREADER_KEY, RFID_READER_DISABLED));
    }
    preferences.end();

    if (firstRun) {
        setRFIDReader(RFID_READER_DISABLED);
        writeEpromSettings();
    }
}

void EVSERFID::validateSettings() {
    // Enable access if no access switch used
    if (RFIDReader != RFID_READER_ENABLE_ONE && evseController.externalSwitch != SWITCH_ACCESS_BUTTON &&
        evseController.externalSwitch != SWITCH_ACCESS_SWITCH) {
        rfidAccessBit = RFID_ACCESS_GRANTED;
    }
}

void EVSERFID::writeEpromSettings() {
    validateSettings();

    Preferences preferences;
    if (preferences.begin(PREFS_RFID_NAMESPACE, false) != true) {
        EVSELogger::error("Unable to write preferences for EVSEModbus");
        return;
    }

    preferences.putUChar(PREFS_RFIDREADER_KEY, RFIDReader);

    preferences.end();
}

void EVSERFID::updateSettings() {
    writeEpromSettings();
}

void EVSERFID::resetSettings() {
    readEpromSettings();
}

bool EVSERFID::isRFIDAccessGranted() {
    return rfidAccessBit == RFID_ACCESS_GRANTED;
}

void EVSERFID::loop() {
    if (!isEnabled()) {
        return;
    }

    // Check if there is a RFID card in front of the reader
    if ((millis() - lastRFIDCheckMillis) >= RFID_CHECK_PERIOD_MILLIS) {
        CheckRFID();
        lastRFIDCheckMillis = millis();
    }

    disableAccessTimeout();
}

void EVSERFID::setup() {
    readEpromSettings();
    // Read all stored RFID's from storage
    ReadRFIDlist();
}

EVSERFID evseRFID;