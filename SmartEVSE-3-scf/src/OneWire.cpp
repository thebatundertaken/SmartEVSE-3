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

#if EVSE_FEATFLAG_ENABLE_RFID
#include "OneWire.h"

#include <Arduino.h>
#include <Preferences.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "EVSEController.h"
#include "EVSELogger.h"
#include "EVSEModbus.h"
#include "EVSEPin.h"
#include "EVSERFID.h"
#include "EVSEScreen.h"
#include "utils.h"

unsigned char RFID[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char RFIDlist[120];  // holds up to 20 RFIDs
Preferences preferences;

// ############################# OneWire functions #############################

// Reset 1-Wire device on SW input
// returns:  1 Device found
//           0 No device found
//         255 Error. Line is pulled low (short, or external button pressed?)
//
unsigned char OneWireReset() {
    unsigned char r;

    if (digitalRead(PIN_SW_IN) == LOW)
        return 255;  // Error, pulled low by external device?

    ONEWIRE_LOW;  // Drive wire low
    delayMicroseconds(480);
    RTC_ENTER_CRITICAL();  // Disable interrupts
    ONEWIRE_FLOATHIGH;     // don't drive high, but use pullup
    delayMicroseconds(70);
    if (digitalRead(PIN_SW_IN) == HIGH)
        r = 0;  // sample pin to see if there is a OneWire device..
    else
        r = 1;
    RTC_EXIT_CRITICAL();  // Restore interrupts
    delayMicroseconds(410);
    return r;
}

void OneWireWriteBit(unsigned char v) {
    if (v & 1) {               // write a '1'
        RTC_ENTER_CRITICAL();  // Disable interrupts
        ONEWIRE_LOW;           // Drive low
        delayMicroseconds(10);
        ONEWIRE_HIGH;         // Drive high
        RTC_EXIT_CRITICAL();  // Restore interrupts
        delayMicroseconds(55);
    } else {                   // write a '0'
        RTC_ENTER_CRITICAL();  // Disable interrupts
        ONEWIRE_LOW;           // Drive low
        delayMicroseconds(65);
        ONEWIRE_HIGH;         // Drive high
        RTC_EXIT_CRITICAL();  // Restore interrupts
        delayMicroseconds(5);
    }
}

unsigned char OneWireReadBit() {
    unsigned char r;

    RTC_ENTER_CRITICAL();  // Disable interrupts
    ONEWIRE_LOW;
    delayMicroseconds(3);
    ONEWIRE_FLOATHIGH;
    delayMicroseconds(10);
    if (digitalRead(PIN_SW_IN) == HIGH)
        r = 1;  // sample pin
    else
        r = 0;
    RTC_EXIT_CRITICAL();  // Restore interrupts
    delayMicroseconds(53);
    return r;
}

void OneWireWrite(unsigned char v) {
    unsigned char bitmask;
    for (bitmask = 0x01; bitmask; bitmask <<= 1) {
        OneWireWriteBit((bitmask & v) ? 1u : 0);
    }
}

unsigned char OneWireRead() {
    unsigned char bitmask, r = 0;

    for (bitmask = 0x01; bitmask; bitmask <<= 1) {
        if (OneWireReadBit())
            r |= bitmask;
    }
    return r;
}

unsigned char OneWireReadCardId() {
    unsigned char x;

    if (OneWireReset() == 1) {  // RFID card detected
        OneWireWrite(0x33);     // OneWire ReadRom Command
        for (x = 0; x < 8; x++)
            RFID[x] = OneWireRead();  // read Family code (0x01) RFID ID (6 bytes) and crc8
        if (crc8(RFID, 8)) {
            RFID[0] = 0;  // CRC incorrect, clear first byte of RFID buffer
            return 0;
        } else {
            for (x = 1; x < 7; x++)
                Serial.printf("%02x", RFID[x]);
            Serial.printf("\r\n");
            return 1;
        }
    }
    return 0;
}

// ############################## RFID functions ##############################

// Read a list of 20 RFID's from preferences
//
void ReadRFIDlist() {
    uint8_t initialized = 0;

    if (preferences.begin("RFIDlist", false)) {
        initialized = preferences.getUChar("RFIDinit", 0);
        if (initialized) {
            // read 120 bytes from storage
            preferences.getBytes("RFID", RFIDlist, 120);
        }
        preferences.end();

        if (initialized == 0) {
            // when unitialized, delete all cardIDs
            DeleteAllRFID();
        }

    } else {
        Serial.print("Error opening preferences!\n");
    }
}

// Write a list of 20 RFID's to the eeprom
//
void WriteRFIDlist() {
    if (preferences.begin("RFIDlist", false)) {
        preferences.putBytes("RFID", RFIDlist, 120);  // write 120 bytes to storage
        preferences.putUChar("RFIDinit", 1);          // data initialized
        preferences.end();
    } else {
        Serial.print("Error opening preferences!\n");
    }
}

// scan for matching RFID in RFIDlist
// returns offset+6 when found, 0 when not found
unsigned char MatchRFID() {
    unsigned char offset = 0, r;

    do {
        r = memcmp(RFID + 1, RFIDlist + offset, 6);  // compare read RFID with list of stored RFID's
        offset += 6;
    } while (r != 0 && offset < 114);

    if (r == 0)
        return offset;  // return offset + 6 in RFIDlist
    else
        return 0;
}

// Store RFID card in memory and eeprom
// returns 1 when successful
// returns 2 when already stored
// returns 0 when all slots are full.
unsigned char StoreRFID() {
    unsigned char offset = 0, r;
    unsigned char empty[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

    // first check if the Card ID was already stored.
    if (MatchRFID())
        return 2;  // already stored, that's ok.

    do {
        r = memcmp(empty, RFIDlist + offset, 6);
        offset += 6;
    } while (r != 0 && offset < 120);
    if (r != 0) {
        // no more room to store RFID
        return 0;
    }
    offset -= 6;
    memcpy(RFIDlist + offset, RFID + 1, 6);

    WriteRFIDlist();
    return 1;
}

// Delete RFID card in memory and eeprom
// returns 1 when successful, 0 when RFID was not found
unsigned char DeleteRFID() {
    unsigned char offset = 0, r;

    offset = MatchRFID();  // find RFID in list
    if (offset) {
        offset -= 6;
        for (r = 0; r < 6; r++)
            RFIDlist[offset + r] = 0xff;
    } else
        return 0;

    //    Serial.printf("deleted %u ",offset);
    //    for (r=0; r<120; r++) Serial.printf("%02x",RFIDlist[r]);

    WriteRFIDlist();
    return 1;
}

void DeleteAllRFID() {
    for (unsigned char i = 0; i < 120; i++) {
        RFIDlist[i] = 0xff;
    }
    WriteRFIDlist();
    evseRFID.setRFIDReader(RFID_READER_DISABLED);
}

void CheckRFID() {
    // When RFID is enabled, a OneWire RFID reader is expected on the SW input
    // RFID Reader set to Enabled, Learn or Delete
    if (evseRFID.RFIDReader == RFID_READER_DISABLED) {
        return;
    }

    // Read card ID
    if (!OneWireReadCardId()) {
        evseRFID.RFIDstatus = RFID_STATUS_NONE;
        return;
    }

    unsigned char x;
    static unsigned char cardoffset = 0;
    switch (evseRFID.RFIDReader) {
        case RFID_READER_ENABLED:
            x = MatchRFID();
            if (x && !evseRFID.RFIDstatus) {
                EVSELogger::info("[OneWire] RFID card found!");
                if (evseRFID.isRFIDAccessGranted()) {
                    // Access Off, Switch back to state B1/C1
                    evseController.setAccess(false);
                } else {
                    evseRFID.rfidAccessBit = RFID_ACCESS_GRANTED;
                }

                evseRFID.RFIDstatus = RFID_STATUS_CARDFOUND;
            } else if (!x) {
                evseRFID.RFIDstatus = RFID_STATUS_INVALIDCARD;
            }
            evseScreen.lightUp();
            break;

        case RFID_READER_ENABLE_ONE:
            x = MatchRFID();
            if (x && !evseRFID.RFIDstatus) {
                EVSELogger::info("[OneWire] RFID card found!");
                if (!evseRFID.isRFIDAccessGranted()) {
                    cardoffset = x;                                // store cardoffset from current card
                    evseRFID.rfidAccessBit = RFID_ACCESS_GRANTED;  // Access On
                } else if (cardoffset == x) {
                    // Access Off, Switch back to state B1/C1
                    evseController.setAccess(false);
                }
                evseRFID.RFIDstatus = RFID_STATUS_CARDFOUND;
            } else if (!x) {
                evseRFID.RFIDstatus = RFID_STATUS_INVALIDCARD;
            }
            evseScreen.lightUp();
            break;

        case RFID_READER_LEARN:
            x = StoreRFID();
            if (x == 1) {
                EVSELogger::info("[OneWire] RFID card stored!");
                evseRFID.RFIDstatus = RFID_STATUS_CARDSTORED;
            } else if (x == 2 && !evseRFID.RFIDstatus) {
                EVSELogger::info("[OneWire] RFID card was already stored!");
                evseRFID.RFIDstatus = RFID_STATUS_CARDSTOREDALREADY;
            } else if (!evseRFID.RFIDstatus) {
                EVSELogger::error("[OneWire] RFID storage full! Delete card first");
                evseRFID.RFIDstatus = RFID_STATUS_CARDSTORAGEFULL;
            }
            break;

        case RFID_READER_DELETE:
            x = DeleteRFID();
            if (x) {
                EVSELogger::info("[OneWire] RFID card deleted!");
                evseRFID.RFIDstatus = RFID_STATUS_CARDDELETED;
            } else if (!evseRFID.RFIDstatus) {
                EVSELogger::warn("[OneWire] RFID card not in list!");
                evseRFID.RFIDstatus = RFID_STATUS_CARDUNKNOWN;
            }
            break;

        default:
            break;
    }
}
#endif
