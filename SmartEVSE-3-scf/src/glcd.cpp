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

#include "glcd.h"

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <string.h>

#include "EVSECluster.h"
#include "EVSEController.h"
#include "EVSEMenu.h"
#include "EVSEPin.h"
#include "EVSERFID.h"
#include "EVSEScreen.h"
#include "EVSEWifi.h"
#include "font.cpp"
#include "font2.cpp"
#include "i18n.h"
#include "main.h"
#include "utils.h"

#define _A0_0 digitalWrite(PIN_LCD_A0_B2, LOW);
#define _A0_1 digitalWrite(PIN_LCD_A0_B2, HIGH);
#define _RSTB_0 digitalWrite(PIN_LCD_RST, LOW);
#define _RSTB_1 digitalWrite(PIN_LCD_RST, HIGH);

const unsigned char LCD_Flow[] = {
    0x00, 0x00, 0x98, 0xCC, 0x66, 0x22, 0x22, 0x22, 0xF2, 0xAA, 0x26, 0x2A, 0xF2, 0x22, 0x22, 0x22, 0x66, 0xCC, 0x88,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x80, 0xC0, 0x60, 0x30, 0x60, 0xC0, 0x90, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x42,
    0x04, 0xE0, 0x10, 0x08, 0x0B, 0x08, 0x10, 0xE0, 0x04, 0x42, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x22, 0x41,
    0x4F, 0x49, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x61, 0x31, 0x18, 0x08,
    0x08, 0x08, 0x08, 0xFF, 0x08, 0x8D, 0x4A, 0xFF, 0x08, 0x08, 0x08, 0x08, 0x18, 0x31, 0x61, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x60, 0x30, 0x18, 0x0C, 0x06, 0x03, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x03, 0x06, 0x0C, 0x19, 0x32, 0x64, 0xC8, 0x10, 0x00, 0x00, 0x08, 0x04, 0x00, 0x01, 0x02, 0x1A,
    0x02, 0x01, 0x00, 0x04, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x05,
    0x88, 0x50, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xFF, 0x00, 0xF8, 0x08, 0x08, 0x08, 0x08, 0xF8, 0x00, 0x00, 0x00, 0xF0, 0x10, 0x10, 0x10, 0x10,
    0x10, 0xF0, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x60, 0x10, 0x08, 0x04, 0x02, 0x82,
    0x81, 0x81, 0x81, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x04, 0x84, 0x8C, 0x88, 0x88, 0x10, 0x10,
    0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x40, 0x60, 0x30, 0x18, 0x0C, 0x07, 0x05, 0x04, 0x04, 0x07, 0x0C, 0x18,
    0x30, 0x68, 0x48, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
    0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x7F, 0x40,
    0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x40, 0x40, 0x40, 0x7F, 0x40, 0x40, 0x40, 0x42, 0x40, 0x7F, 0x40, 0x40, 0x7F,
    0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
    0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x06, 0x19, 0x10, 0x10, 0x1C, 0x02, 0x19, 0x24, 0x42, 0x42, 0x24, 0x19, 0x02,
    0x1C, 0x10, 0x10, 0x10, 0x10, 0x1C, 0x02, 0x19, 0x24, 0x42, 0x42, 0x24, 0x19, 0x02, 0x1C, 0x10, 0x10, 0x1F};

// Toggle display between two values
unsigned long LCDToggleMillis = millis();
// Cycle through text messages
unsigned long LCDTextCircleMillis = millis();
unsigned int GLCDx, GLCDy;
char LCDStr[24];
unsigned char energy_mains = 20;  // X position
unsigned char energy_ev = 74;     // X position
uint8_t GLCDbuf[512];             // GLCD buffer (half of the display)

void st7565_command(unsigned char data) {
    _A0_0;
    SPI.transfer(data);
}

void st7565_data(unsigned char data) {
    _A0_1;
    SPI.transfer(data);
}

void goto_row(unsigned char y) {
    unsigned char pattern;
    pattern = 0xB0 | (y & 0xBF);  // put row address on data port set command
    st7565_command(pattern);
}
//--------------------

void goto_col(unsigned char x) {
    unsigned char pattern;
    pattern = ((0xF0 & x) >> 4) | 0x10;
    st7565_command(pattern);  // set high byte column command
    pattern = ((0x0F & x)) | 0x00;
    st7565_command(pattern);  // set low byte column command;
}
//--------------------

void goto_xy(unsigned char x, unsigned char y) {
    goto_col(x);
    goto_row(y);
}

void glcd_clrln(unsigned char ln, unsigned char data) {
    goto_xy(0, ln);
    for (unsigned char i = 0; i < 128; i++) {
        st7565_data(data);  // put data on data port
    }
}

void glcd_clear() {
    for (unsigned char i = 0; i < 8; i++) {
        glcd_clrln(i, 0);
    }
}

void GLCD_buffer_clr() {
    unsigned char x = 0;
    do {
        GLCDbuf[x++] = 0;  // clear first 256 bytes of GLCD buffer
    } while (x != 0);
}

void GLCD_sendbuf(unsigned char RowAdr, unsigned char Rows) {
    unsigned char i, y = 0;
    unsigned int x = 0;

    do {
        goto_xy(0, RowAdr + y);
        for (i = 0; i < 128; i++)
            st7565_data(GLCDbuf[x++]);  // put data on data port
    } while (++y < Rows);
}

void GLCD_font_condense(unsigned char c, unsigned char* start, unsigned char* end, unsigned char space) {
    if (c >= '0' && c <= '9')
        return;

    if (c == ' ' && space)
        return;

    if (font[c][0] == 0) {
        if (font[c][1] == 0)
            *start = 2;
        else
            *start = 1;
    }

    if (font[c][4] == 0)
        *end = 4;
}

unsigned char GLCD_text_length(const char* str) {
    unsigned char i = 0, length = 0;
    unsigned char s, e;

    while (str[i]) {
        s = 0;
        e = 5;
        GLCD_font_condense(str[i], &s, &e, 0);
        length += (e - s) + 1;
        i++;
    }

    return length - 1;
}

unsigned char GLCD_text_length2(const char* str) {
    unsigned char i = 0, length = 0;

    while (str[i]) {
        length += font2[str[i]][0] + 2;
        i++;
    }

    return length - 2;
}

/**
 * Write character to buffer
 *
 * @param c
 * @param 00000000 Options
 *            |+++ Shift font down
 *            +--- Merge with buffer content
 */
void GLCD_write_buf(unsigned int c, unsigned char Options) {
    unsigned int f, x;
    unsigned char i = 0, m = 5, shift = 0;
    bool merge = false;

    x = 128 * GLCDy;
    x += GLCDx;

    GLCD_font_condense(c, &i, &m, 1);  // remove whitespace from font
    GLCDx += (m - i) + 1;

    if (Options) {
        shift = Options & 0b00000111;
        if (Options & GLCD_MERGE) {
            merge = true;
        }
    }

    do {
        f = font[c][i];
        if (shift) {
            f <<= shift;
        }
        if (merge) {
            f |= GLCDbuf[x];
        }
        GLCDbuf[x] = f;
        x++;
    } while (++i < m);
}

// Write one double height character to the GLCD buffer
// special characters '.' and ' ' will use reduced width in the buffer
void GLCD_write_buf2(unsigned int c) {
    unsigned char i = 1;

    while ((i < (font2[c][0] * 2)) && (GLCDx < 128)) {
        GLCDbuf[GLCDx] = font2[c][i];
        GLCDbuf[GLCDx + 128] = font2[c][i + 1];
        i += 2;
        GLCDx++;
    }

    GLCDx += 2;
}

/**
 * Write a string to LCD buffer
 *
 * @param str
 * @param x
 * @param y
 * @param Options
 */
void GLCD_write_buf_str(unsigned char x, unsigned char y, const char* str, unsigned char Options) {
    unsigned char i = 0;

    switch (Options) {
        case GLCD_ALIGN_LEFT:
            GLCDx = x;
            break;
        case GLCD_ALIGN_CENTER:
            GLCDx = x - (GLCD_text_length(str) / 2);
            break;
        case GLCD_ALIGN_RIGHT:
            GLCDx = x - GLCD_text_length(str);
            break;
    }

    GLCDy = y;
    while (str[i]) {
        GLCD_write_buf(str[i++], 0);
    }
}

void GLCD_write_buf_str2(const char* str, unsigned char Options) {
    unsigned char i = 0;

    if (Options == GLCD_ALIGN_CENTER) {
        GLCDx = 64 - GLCD_text_length2(str) / 2;
    } else {
        GLCDx = 2;
    }

    while (str[i]) {
        GLCD_write_buf2(str[i++]);
    }
}

void GLCD_print_buf(unsigned char y, const char* str) {
    GLCD_buffer_clr();  // Clear buffer
    GLCD_write_buf_str(0, y, str, GLCD_ALIGN_LEFT);
    GLCD_sendbuf(y, 1);  // copy buffer to LCD
}

// uses buffer
void GLCD_print_buf2_left(const char* data) {
    GLCD_buffer_clr();  // Clear buffer
    GLCD_write_buf_str2(data, GLCD_ALIGN_LEFT);
    GLCD_sendbuf(2, 2);  // copy buffer to LCD
}

void GLCD_print_buf2(unsigned char y, const char* str) {
    GLCD_buffer_clr();  // Clear buffer
    GLCD_write_buf_str2(str, GLCD_ALIGN_CENTER);
    GLCD_sendbuf(y, 2);  // copy buffer to LCD
}

bool getLCDToggle() {
    return (((millis() - LCDToggleMillis) / 1000) % 2) == 0;
}

// Write Menu to buffer, then send to GLCD
void GLCD_print_menu(unsigned char y, const char* str) {
    GLCD_buffer_clr();  // Clear buffer
    GLCD_write_buf_str2(str, GLCD_ALIGN_CENTER);

    if ((evseMenu.subMenu && y == 4) || (!evseMenu.subMenu && y == 2)) {  // navigation arrows
        GLCDx = 0;
        GLCD_write_buf2('<');
        GLCDx = 10 * 12;  // last character of line
        GLCD_write_buf2('>');
    }

    GLCD_sendbuf(y, 2);
}

// uses buffer
// Display/Scroll helptext on LCD
void GLCDMenuItemHelp() {
    unsigned int x;

    x = strlen(menuEntries[evseMenu.currentMenuOption].Desc);
    GLCD_print_buf2_left(menuEntries[evseMenu.currentMenuOption].Desc + evseScreen.LCDpos);

    if (evseScreen.LCDpos++ == 0) {
        evseScreen.ScrollTimerHelpMenu = millis() - 4000;
    } else if (evseScreen.LCDpos > (x - 10)) {
        evseScreen.ScrollTimerHelpMenu = millis() - 3000;
        evseScreen.LCDpos = 0;
    } else {
        evseScreen.ScrollTimerHelpMenu = millis() - 4700;
    }
}

void GLCDControllerNoCTCommError() {
    GLCD_print_buf2(0, (const char*)I18N_ERROR_NOSERIALCOM1);
    GLCD_print_buf2(2, (const char*)I18N_ERROR_NOSERIALCOM2);
    GLCD_print_buf2(4, (const char*)I18N_ERROR_NOSERIALCOM3);
    GLCD_print_buf2(6, (const char*)I18N_ERROR_NOSERIALCOM4);
}

void GLCDControllerTempHighError() {
    GLCD_print_buf2(0, (const char*)I18N_ERROR_TEMP1);
    GLCD_print_buf2(2, (const char*)I18N_ERROR_TEMP2);
    GLCD_print_buf2(4, (const char*)I18N_ERROR_TEMP3);
    GLCD_print_buf2(6, (const char*)I18N_ERROR_TEMP4);
}

void GLCDControllerOTAFlashError() {
    GLCD_print_buf2(2, (const char*)I18N_ERROR_OTA1);
    GLCD_print_buf2(4, (const char*)I18N_ERROR_OTA2);
}

void GLCDControllerRCMError() {
    if (!getLCDToggle()) {
        GLCD_print_buf2(0, (const char*)I18N_ERROR_RCM1);
        GLCD_print_buf2(2, (const char*)I18N_ERROR_RCM2);
        GLCD_print_buf2(4, (const char*)I18N_ERROR_RCM3);
        GLCD_print_buf2(6, (const char*)I18N_ERROR_RCM4);
    } else {
        GLCD_print_buf2(0, (const char*)I18N_ERROR_RCM5);
        GLCD_print_buf2(2, (const char*)I18N_ERROR_RCM6);
        GLCD_print_buf2(4, (const char*)I18N_ERROR_RCM7);
        GLCD_print_buf2(6, (const char*)I18N_ERROR_RCM8);
    }
}

void GLCDRFID() {
    switch (evseRFID.RFIDstatus) {
        case RFID_STATUS_CARDSTORED:
            GLCD_print_buf(0, (const char*)I18N_RFID_CARDSTORED);
            break;

        case RFID_STATUS_CARDDELETED:
            GLCD_print_buf(0, (const char*)I18N_RFID_CARDDELETED);
            break;

        case RFID_STATUS_CARDSTOREDALREADY:
            GLCD_print_buf(0, (const char*)I18N_RFID_CARDALREADY);
            break;

        case RFID_STATUS_CARDUNKNOWN:
            GLCD_print_buf(0, (const char*)I18N_RFID_CARDNOTFOUND);
            break;

        case RFID_STATUS_CARDSTORAGEFULL:
            GLCD_print_buf(0, (const char*)I18N_RFID_STORAGEFULL);
            break;

        default:
            // Clear line
            glcd_clrln(0, 0x00);
            break;
    }

    // reset timer, so it will not exit the menu when learning/deleting cards
    evseScreen.lightUp();
    evseMenu.resetInactivityTimer();
}

void GLCDWifiPortalCountdown() {
    glcd_clrln(0, 0);
    // Bottom row of the GLCD
    GLCD_buffer_clr();

    sprintf(LCDStr, "WiFi portal start in %u", evseWifi.getPortalCountdownSeconds());
    GLCD_write_buf_str(0, 0, LCDStr, GLCD_ALIGN_LEFT);

    GLCD_sendbuf(0, 1);
}

uint8_t GLCDAccessPoint_toggle = 0;
void GLCDAccessPoint() {
    // Top row of the GLCD
    glcd_clrln(0, 0);
    // Bottom row of the GLCD
    GLCD_buffer_clr();

    // Show Access Point name and password
    if (GLCDAccessPoint_toggle++ < 2) {
        sprintf(LCDStr, "Wifi: %s", evseWifi.getApHostname());
        GLCD_write_buf_str(0, 0, LCDStr, GLCD_ALIGN_LEFT);
    } else {
        sprintf(LCDStr, "Pass: %s", evseWifi.getApPassword());
        GLCD_write_buf_str(127, 0, LCDStr, GLCD_ALIGN_RIGHT);
        if (GLCDAccessPoint_toggle++ > 5) {
            GLCDAccessPoint_toggle = 0;
        }
    }

    GLCD_sendbuf(0, 1);
}

void GLCDWifiInfo() {
    // Displays IP and time in top row

    // Bottom row of the GLCD
    GLCD_buffer_clr();

    if (WiFi.status() == WL_CONNECTED) {
        // Display IP Address
        IPAddress localIp = evseWifi.getLlocalIp();
        sprintf(LCDStr, "%u.%u.%u.%u", localIp[0], localIp[1], localIp[2], localIp[3]);
        GLCD_write_buf_str(0, 0, LCDStr, GLCD_ALIGN_LEFT);

        // Display local time
        uint16_t localTime = evseWifi.getNTPLocalTime();
        if (localTime != UINT16_MAX) {
            sprintf(LCDStr, "%02u:%02u", localTime / 100u, localTime % 100);
        } else {
            sprintf(LCDStr, "--:--");
        }

        GLCD_write_buf_str(127, 0, LCDStr, GLCD_ALIGN_RIGHT);
        GLCD_sendbuf(0, 1);
    } else {
        GLCD_write_buf_str(0, 0, "Not connected to WiFi", GLCD_ALIGN_LEFT);
        GLCD_sendbuf(0, 1);
    }
}

void GLCDRemoveSunFromBuffer() {
    for (uint8_t x = 0; x < 13; x++) {
        GLCDbuf[x + 74u] = 0;
        GLCDbuf[x + 74u + 128u] = 0;
    }
}

void GLCDRemoveClockFromBuffer() {
    for (uint8_t x = 0; x < 8; x++) {
        GLCDbuf[x + 92u] = 0;
    }
}

void GLCDRemoveLineBetweenHouseAndCar() {
    for (uint8_t x = 73; x < 96; x++) {
        GLCDbuf[3u * 128u + x] = 0;
    }
}

void GLCDAnimateMainsEnergyFlow() {
    const int16_t irmsSum = evseController.getMainsMeasuredCurrent(true);

    if (irmsSum < 0) {
        // negative => solar surplus, animate from house to grid (-<-<-<-)
        energy_mains -= 3;
        if (energy_mains < 20) {
            // Only in Mode: Smart or Solar
            energy_mains = 44;
        }
    } else {
        // positive => grid power consumption, animate from grid to house (->->->-)
        energy_mains += 3;
        if (energy_mains > 44) {
            energy_mains = 20;
        }
    }

    GLCDx = energy_mains;
    GLCDy = 3;

    // Do not render energy flow 'blob' for small current flows (<= 0.3A)
    if (abs(irmsSum) > 3) {
        GLCD_write_buf(0x0A, 0);
    }
}

void GLCDAnimateEVEnergyFlow() {
    // animate energy flow to EV
    energy_ev += 3;
    if (energy_ev > 89) {
        energy_ev = 74;
    }

    GLCDx = energy_ev;
    GLCDy = 3;
    // Show energy flow 'blob' between House and Car
    GLCD_write_buf(0x0A, 0);

    if (getLCDToggle() && evseModbus.evMeter != EV_METER_DISABLED) {
        if (evseModbus.powerMeasured < 9950) {
            sprintfl(LCDStr, I18N_POWERMEASURED_FORMAT_SHORT, evseModbus.powerMeasured, 3, 1);
        } else {
            sprintfl(LCDStr, I18N_POWERMEASURED_FORMAT_LONG, evseModbus.powerMeasured, 3, 0);
        }
    } else {
        // sprintfl(LCDStr, I18N_POWERAMPS_FORMAT, evseModbus.balancedChargeCurrent[0], 1,
        // 0);
        sprintfl(LCDStr, I18N_POWERAMPS_FORMAT, evseController.getChargeCurrent(), 1, 0);
    }
    GLCD_write_buf_str(85, 2, LCDStr, GLCD_ALIGN_CENTER);
}

void circleChargingMessage() {
    // Cycle charging message every 1 sec
    switch (((millis() - LCDTextCircleMillis) / 1000) % 5) {
        case 1:
            GLCD_print_buf2(5, (const char*)I18N_STATE_CHARGING);
            break;

        case 2:
            if (evseModbus.evMeter != EV_METER_DISABLED) {
                sprintfl(LCDStr, I18N_STATE_CHARGINGPOWER, evseModbus.powerMeasured, 3, 1);
                GLCD_print_buf2(5, LCDStr);
            }
            break;

        case 3:
            if (evseModbus.evMeter != EV_METER_DISABLED) {
                sprintfl(LCDStr, I18N_STATE_CHARGINGENERGY, evseModbus.energyCharged, 3, 2);
                GLCD_print_buf2(5, LCDStr);
            }
            break;

        case 4:
            // sprintf(LCDStr, I18N_STATE_CHARGING2, evseModbus.balancedChargeCurrent[0] /
            // 10, evseModbus.balancedChargeCurrent[0] % 10);
            sprintf(LCDStr, I18N_STATE_CHARGING2, evseController.getChargeCurrent() / 10,
                    evseController.getChargeCurrent() % 10);
            GLCD_print_buf2(5, LCDStr);
            break;

        default:
            if (evseController.mode != MODE_NORMAL) {
                GLCD_print_buf2(5,
                                (const char*)(evseController.mode == MODE_SOLAR) ? I18N_MODE_SOLAR : I18N_MODE_SMART);
            }
    }
}

void GLCDSmartSolarMode() {
    // copy Flow Menu to LCD buffer
    memcpy(GLCDbuf, LCD_Flow, 512);

    if (evseController.mode == MODE_SMART) {
        // remove the Sun from the LCD buffer
        GLCDRemoveSunFromBuffer();
        // remove the clock (used for solarStopTimer)
        GLCDRemoveClockFromBuffer();
    } else {
        // MODE_SOLAR
        if (evseController.solarStopTimer > 0) {
            // display remaining time before charging is stopped
            unsigned int seconds = evseController.solarStopTimer;
            sprintf(LCDStr, I18N_SOLARSTOPTIMER_FORMAT, (int)(seconds / 60), (seconds % 60));
            GLCD_write_buf_str(100, 0, LCDStr, GLCD_ALIGN_LEFT);
        } else {
            // remove the clock from the LCD buffer
            GLCDRemoveClockFromBuffer();
        }
    }

    GLCDAnimateMainsEnergyFlow();

    // If we have a EV kWh meter configured, Show total charged energy in kWh on LCD
    if (evseModbus.evMeter != EV_METER_DISABLED) {
        sprintfl(LCDStr, I18N_ENERGYCHARGED_FORMAT, evseModbus.energyCharged, 3, 1);
        GLCD_write_buf_str(89, 1, LCDStr, GLCD_ALIGN_LEFT);
    }

    bool LCDToggle = getLCDToggle();
    switch (evseController.state) {
        case STATE_A_STANDBY:
            GLCDRemoveLineBetweenHouseAndCar();
            break;

        case STATE_C_CHARGING:
            GLCDAnimateEVEnergyFlow();
            break;
    }

    // Show Sum of currents when solar charging.
    if (LCDToggle && evseController.mode == MODE_SOLAR) {
        GLCDx = 41;
        GLCDy = 0;
        // Sum symbol
        GLCD_write_buf(0x0B, 0);

        // Display total solar surplus (absolute value)
        int16_t imeasured = abs(evseController.getMainsMeasuredCurrent(true));
        sprintf(LCDStr, I18N_CURRENTS_FORMAT_SOLAR, imeasured / 10, imeasured % 10);
        GLCD_write_buf_str(48, 1, LCDStr, GLCD_ALIGN_RIGHT);
    } else {
        // Display L1, L2 and L3 currents on LCD
        for (unsigned char phase = 0; phase < 3; phase++) {
            sprintf(LCDStr, I18N_CURRENTS_FORMAT_SMART, (int)(evseController.Irms[phase] / 10),
                    (unsigned int)abs(evseController.Irms[phase]) % 10);
            GLCD_write_buf_str(48, phase, LCDStr, GLCD_ALIGN_RIGHT);
        }
    }
    GLCD_sendbuf(0, 4);

    glcd_clrln(4, 0);
    if (evseController.errorFlags & ERROR_FLAG_LESS_6A) {
        // No power error
        if (!LCDToggle) {
            GLCD_print_buf2(5, (const char*)I18N_ERROR_LESS6A1);
        } else {
            GLCD_print_buf2(5, (const char*)I18N_ERROR_LESS6A2);
        }
    } else if (evseController.errorFlags & ERROR_FLAG_NO_SUN) {
        // No sun error
        GLCD_print_buf2(5, (const char*)LCDToggle ? I18N_ERROR_NOSUN2 : I18N_ERROR_NOSUN1);
    } else if (evseController.state != STATE_C_CHARGING) {
        if (evseController.isChargeDelayed()) {
            // Charge delayed
            sprintf(LCDStr, I18N_STATE_READY1DELAY, evseController.getChargeDelaySeconds());
            GLCD_print_buf2(5, LCDStr);
        } else {
            // Ready to charge
            GLCD_print_buf2(5, (const char*)I18N_STATE_READY1NODELAY);
        }
    } else if (evseController.state == STATE_C_CHARGING) {
        circleChargingMessage();
    }

    glcd_clrln(7, 0x00);
}

void GLCDNormalMode() {
    glcd_clrln(0, 0x00);
    glcd_clrln(1, 0x04);  // horizontal line
    glcd_clrln(6, 0x10);  // horizontal line
    glcd_clrln(7, 0x00);

    if (evseController.errorFlags & ERROR_FLAG_LESS_6A) {
        GLCD_print_buf2(2, (const char*)I18N_ERROR_LESS6A1);
        GLCD_print_buf2(4, (const char*)I18N_ERROR_LESS6A2);
        return;
    }

    if (evseController.state == STATE_C_CHARGING) {
        GLCD_print_buf2(2, (const char*)I18N_STATE_CHARGING);
        // sprintf(LCDStr, I18N_STATE_CHARGING2, evseModbus.balancedChargeCurrent[0] / 10,
        // evseModbus.balancedChargeCurrent[0] % 10);
        sprintf(LCDStr, I18N_STATE_CHARGING2, evseController.getChargeCurrent() / 10,
                evseController.getChargeCurrent() % 10);
        GLCD_print_buf2(4, LCDStr);
        return;
    }

    if (evseRFID.isRFIDAccessGranted()) {
        if (evseController.isChargeDelayed()) {
            // Charge delayed
            GLCD_print_buf2(2, (const char*)I18N_STATE_READY1);
            sprintf(LCDStr, I18N_STATE_READY2DELAY, evseController.getChargeDelaySeconds());
            GLCD_print_buf2(4, LCDStr);
        } else {
            // Ready to charge
            GLCD_print_buf2(2, (const char*)I18N_STATE_READY1);
            GLCD_print_buf2(4, (const char*)I18N_STATE_READY2);
        }

        return;
    }

    if (evseRFID.isEnabled()) {
        switch (evseRFID.RFIDstatus) {
            case RFID_STATUS_INVALIDCARD:
                GLCD_print_buf2(2, (const char*)I18N_RFID_INVALID1);
                GLCD_print_buf2(4, (const char*)I18N_RFID_INVALID2);
                break;

            default:
                GLCD_print_buf2(2, (const char*)I18N_STATE_READYRFIDON1);
                GLCD_print_buf2(4, (const char*)I18N_STATE_READYRFIDON2);
        }
        return;
    }

    GLCD_print_buf2(2, (const char*)I18N_ACCESSDENIED1);
    GLCD_print_buf2(4, (const char*)I18N_ACCESSDENIED2);
}

void GLCDHoldForMenu() {
    glcd_clrln(0, 0x00);
    // horizontal line
    glcd_clrln(1, 0x04);
    GLCD_print_buf2(2, (const char*)I18N_MENU_HOLD2SEC);
    GLCD_print_buf2(4, (const char*)I18N_MENU_HOLDFORMENU);
    // horizontal line
    glcd_clrln(6, 0x10);
    glcd_clrln(7, 0x00);
}

void GLCDMenu() {
    // add navigation arrows on both sides
    GLCD_print_menu(2, menuEntries[evseMenu.currentMenuOption].LCD);

    uint8_t wifimode = evseWifi.getWifiMode();
    // When connected to Wifi, display IP and time in top row
    if (wifimode == WIFI_MODE_ENABLED) {
        GLCDWifiInfo();
    }

    switch (evseMenu.currentMenuOption) {
        case MENU_WIFI:
            if (evseMenu.subMenu && wifimode == WIFI_MODE_DISABLED) {
                // Clear Wifi info from top row of the GLCD
                glcd_clrln(0, 0);
            } else if (evseMenu.subMenu && wifimode == WIFI_MODE_START_PORTAL) {
                if (evseWifi.isPortalReady()) {
                    GLCDAccessPoint();
                } else {
                    GLCDWifiPortalCountdown();
                }
            }

            GLCD_print_menu(4, evseMenu.getMenuItemi18nText(evseMenu.currentMenuOption));

            break;

        case MENU_RFIDREADER:
            if (evseMenu.subMenu) {
                GLCDRFID();
            } else {
                GLCD_print_menu(4, evseMenu.getMenuItemi18nText(evseMenu.currentMenuOption));
            }
            break;

        default:
            // print Menu
            GLCD_print_menu(4, evseMenu.getMenuItemi18nText(evseMenu.currentMenuOption));
            break;
    }

    // Bottom row of the GLCD
    GLCD_buffer_clr();
    // show the internal temperature. ° Degree symbol
    sprintf(LCDStr, I18N_TEMPERATURE_FORMAT, evseController.temperature, 0x0C);
    GLCD_write_buf_str(6, 0, LCDStr, GLCD_ALIGN_LEFT);

    // show navigation position in the menu
    sprintf(LCDStr, I18N_MENUNAVIGATIONPOSITION_FORMAT, evseMenu.getPosInMenu(), evseMenu.getMenuItemsCount());
    GLCD_write_buf_str(64, 0, LCDStr, GLCD_ALIGN_CENTER);

    // show software version in bottom right corner
    sprintf(LCDStr, I18N_SOFTWAREVERSION_FORMAT, (const char*)EVSE_VERSION);
    GLCD_write_buf_str(122, 0, LCDStr, GLCD_ALIGN_RIGHT);
    GLCD_sendbuf(7, 1);
}

void GLCD_init() {
    delay(200);  // transients on the line could have garbled the LCD, wait 200ms
                 // then re-init.
    _A0_0;       // A0=0
    _RSTB_0;     // Reset GLCD module
    delayMicroseconds(4);
    _RSTB_1;  // Reset line high
    delayMicroseconds(4);

    st7565_command(0xA2);  // (11) set bias at duty cycle 1.65 (0xA2=1.9 0xA3=1.6)
    st7565_command(0xA0);  // (8) SEG direction (0xA0 or 0xA1)
    st7565_command(0xC8);  // (15) comm direction normal =0xC0 comm reverse= 0xC8

    st7565_command(0x20 | 0x04);  // (17) set Regulation Ratio (0-7)

    st7565_command(0xF8);  // (19) send Booster command
    st7565_command(0x01);  // set Booster value 00=4x 01=5x

    st7565_command(0x81);  // (18) send Electronic Volume command 0x81
    st7565_command(0x24);  // set Electronic volume (0x00-0x3f)

    st7565_command(0xA6);  // (9) Inverse display (0xA7=inverse 0xA6=normal)
    st7565_command(0xA4);  // (10) ALL pixel on (A4=normal, A5=all ON)

    st7565_command(0x28 | 0x07);  // (16) ALL Power Control ON

    glcd_clear();    // clear internal GLCD buffer
    goto_row(0x00);  // (3) Set page address
    goto_col(0x00);  // (4) Set column addr LSB

    st7565_command(0xAF);  // (1) ON command
}
