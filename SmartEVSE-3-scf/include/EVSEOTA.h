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

#ifndef __EVSEOTA
#define __EVSEOTA

#include <ESPAsyncWebServer.h>

// Partition size is 0x90000
#define SPI_PARTITION_SIZE 0x90000

class EVSEOTA {
   public:
    static void updateGETRequestHandler(AsyncWebServerRequest* request);
    static void updatePOSTRequestHandler(AsyncWebServerRequest* request);
    static void updateMultipartUploadHandler(AsyncWebServerRequest* request,
                                             String filename,
                                             size_t index,
                                             uint8_t* data,
                                             size_t len,
                                             bool final);
};

#endif