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

#ifndef __EVSE_MAIN
#define __EVSE_MAIN

#ifndef EVSE_LITTLE
#define EVSE_LITTLE 0
#endif

#ifndef EVSE_FEATFLAG_ENABLE_RFID
#define EVSE_FEATFLAG_ENABLE_RFID 1
#endif

#ifndef EVSE_FEATFLAG_ENABLE_LEDS
#define EVSE_FEATFLAG_ENABLE_LEDS 1
#endif

#ifndef EVSE_FEATFLAG_ENABLE_RCMON
#define EVSE_FEATFLAG_ENABLE_RCMON 1
#endif

#ifndef EVSE_FEATFLAG_ENABLE_EXTSWITCH
#define EVSE_FEATFLAG_ENABLE_EXTSWITCH 1
#endif

#ifndef EVSE_FEATFLAG_ENABLE_POWERSHARE
#define EVSE_FEATFLAG_ENABLE_POWERSHARE 1
#endif

#ifndef EVSE_FEATFLAG_ENABLE_EVMETER
#define EVSE_FEATFLAG_ENABLE_EVMETER 1
#endif

// SmartEVSE software version
#define EVSE_VERSION "4.0.25"

#endif