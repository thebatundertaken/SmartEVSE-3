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

#ifndef __EVSERFID
#define __EVSERFID

#include <stdio.h>

// Seconds delay for the EVSE to revoke access
#define RFID_REVOKE_ACCESS_TIMEOUT_MILLIS 60000

// RFID Reader (0:Disabled / 1:Enabled / 2:Enable One / 3:Learn / 4:Delete /
// 5:Delete All)
#define RFID_READER_DISABLED 0
// Enable: All learned cards accepted for locking /unlocking
#define RFID_READER_ENABLED 1
// EnableOne: Only the card that unlocks, can re-lock the EVSE
#define RFID_READER_ENABLE_ONE 2
#define RFID_READER_LEARN 3
#define RFID_READER_DELETE 4
#define RFID_READER_DELETE_ALL 5

#define RFID_NO_ACCESS 0
#define RFID_ACCESS_GRANTED 1

#define RFID_STATUS_NONE 0
#define RFID_STATUS_CARDFOUND 1
#define RFID_STATUS_CARDSTORED 2
#define RFID_STATUS_CARDDELETED 3
#define RFID_STATUS_CARDSTOREDALREADY 4
#define RFID_STATUS_CARDUNKNOWN 5
#define RFID_STATUS_CARDSTORAGEFULL 6
#define RFID_STATUS_INVALIDCARD 7

#define RFID_CHECK_PERIOD_MILLIS 1000

class EVSERFID {
 public:
  EVSERFID(){};
  void setup();
  void loop();
  bool isEnabled() { return RFIDReader != RFID_READER_DISABLED; };
  bool isRFIDAccessGranted();
  void updateSettings();
  void resetSettings();
  void setRFIDReader(uint8_t value);

  //  1 => RFID ok, access granted; 0 => false
  uint8_t rfidAccessBit = RFID_NO_ACCESS;
  uint8_t RFIDstatus = RFID_STATUS_NONE;
  uint8_t RFIDReader = RFID_READER_DISABLED;

 private:
  void readEpromSettings();
  void writeEpromSettings();
  void validateSettings();
  void disableAccessTimeout();
  unsigned long lastRFIDCheckMillis = 0;
  // Timer to re-lock the EVSE (and unlock the cable) after 60 seconds
  unsigned long rfidLockWaitMillis = 0;
};

extern EVSERFID evseRFID;
#endif