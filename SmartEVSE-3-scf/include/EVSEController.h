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

#ifndef __EVSECONTROLLER
#define __EVSECONTROLLER

#include "esp_adc_cal.h"

#define CONFIG_SOCKET 0
#define CONFIG_FIXED_CABLE 1

#define RC_MON_DISABLED 0  // Residual Current Monitoring on IO3. Disabled=0, RCM14=1
#define RC_MON_ENABLED 1   // Residual Current Monitoring on IO3. Disabled=0, RCM14=1

#define ICAL_DEFAULT 1024  // Irms Calibration value (for Current transformers)

#define ADC_SAMPLES_SIZE 25

// ERROR FLAGS
#define ERROR_FLAG_NO_ERROR 0
#define ERROR_FLAG_LESS_6A 1
#define ERROR_FLAG_CT_NOCOMM 2
// Temperature reached 65C
#define ERROR_FLAG_TEMP_HIGH 4
// #define ERROR_FLAG_UNUSED 8       // Unused
// RCM tripped. >6mA DC residual current detected.
#define ERROR_FLAG_RCM_TRIPPED 16
#define ERROR_FLAG_NO_SUN 32
// Bootloader update error
#define ERROR_FLAG_BL_FLASH 128

// Degrees
#define DEFAULT_MAX_TEMPERATURE 65
#define TEMPERATURE_COOLDOWN_THRESHOLD 10
#define TEMPERATURE_SAMPLE_PERIOD_MILLIS 1000

#define CONTROL_PILOT_12V 1
#define CONTROL_PILOT_9V 2
#define CONTROL_PILOT_6V 3
#define CONTROL_PILOT_DIODE_CHECK_OK 4
#define CONTROL_PILOT_NOK 0

// EVSE mode settings
#define MODE_NORMAL 0
#define MODE_SMART 1
#define MODE_SOLAR 2

#define STATE_A_STANDBY 0           // A Vehicle not detected
#define STATE_B_VEHICLE_DETECTED 1  // B Vehicle detected / not ready to accept energy
#define STATE_C_CHARGING 2          // C Vehicle detected / ready to accept energy / ventilation not required
// #define STATE_D_WITH_VENTILATION 3         // D Vehicle detected / ready to accept energy / ventilation required (not implemented)
// #define STATE_E_NO_POWER 11                // E No power (shut off)
// #define STATE_F_ERROR 12                   // F Error
#define STATE_MODBUS_COMM_B 4                 // E State change request A->B (set by node)
#define STATE_MODBUS_COMM_B_OK 5              // F State change A->B OK (set by master)
#define STATE_MODBUS_COMM_C 6                 // G State change request B->C (set by node)
#define STATE_MODBUS_COMM_C_OK 7              // H State change B->C OK (set by master)
#define STATE_DISCONNECT_IN_PROGRESS 8        // I Activation mode in progress
#define STATE_B1_VEHICLE_DETECTED_NO_POWER 9  // J Vehicle detected / no PWM signal
#define STATE_C1_CHARGING_NO_POWER 10         // K Vehicle charging / no PWM signal (temp state when stopping charge from EVSE)
#define NOSTATE 255

// Let car 3 seconds to process electric signal on the fly before disconnecting
#define DISCONNECT_WAIT_MILLIS 3000
// Turn off current if unused for 30 seconds
#define CURRENTAVAILABLE_WAIT_TIMEOUT_MILLIS 30000
// if the EV does not stop charging in 6 seconds, we will open the contactor.
#define CHARGING_NO_POWER_WAIT_TIMEOUT_MILLIS 6000

// Seconds to wait after overcurrent, before trying again
#define DEFAULT_CHARGE_DELAY_SECONDS 60

// minimum Current the EV will accept
#define MIN_CURRENT 6
// max charging Current for the EV (A)
#define MAX_CURRENT 13
// max Current the Mains connection can supply
#define MAX_MAINS 25

// Start charging when surplus current on one phase exceeds 4A (Solar)
#define SOLAR_START_CURRENT 4
// Stop charging after 10 minutes at MIN charge current (Solar)
#define SOLAR_STOP_TIME_MINUTES 10
// Allow the use of grid power when solar charging (Amps)
#define SOLAR_IMPORT_CURRENT 0

// External Switch (0:Disable / 1:Access B / 2:Access S / 3:Smart-Solar B / 4:Smart-Solar S)
// 0= Charge on plugin, 1= (Push)Button on IO2 is used to Start/Stop charging.
#define SWITCH_DISABLED 0
#define SWITCH_ACCESS_BUTTON 1
#define SWITCH_ACCESS_SWITCH 2
#define SWITCH_SMARTSOLAR_BUTTON 3
#define SWITCH_SMARTSOLAR_SWITCH 4

// Smart-Solar Button or hold button for 1,5 second to STOP charging
#define SWITCH_SMARTSOLAR_BUTTON_LONGPRESSED_MILLIS 1500

// TODO: Will be placed in the appropriate position after the rtc module is finished.
extern portMUX_TYPE rtc_spinlock;
#define RTC_ENTER_CRITICAL() portENTER_CRITICAL(&rtc_spinlock)
#define RTC_EXIT_CRITICAL() portEXIT_CRITICAL(&rtc_spinlock)

class EVSEController {
   public:
    EVSEController(){};

    void setup();
    void loop();

    void setMode(uint8_t newMode);
    void switchModeSolarSmart();
    void setState(uint8_t NewState);
    void setAccess(bool access);
    void setCurrent(uint16_t current);
    void calcChargeCurrent();
    bool isChargeDelayed() { return chargeDelaySeconds > 0; };
    bool isVehicleConnected();
    uint8_t getChargeDelaySeconds() { return chargeDelaySeconds; };

    void calculateCalibration(uint16_t CT1);
    void updateSettings();
    void resetSettings();

    // RCM = Residual Current Monitor
    void resetRCMErrorFlag();
    void setSolarStopTimer(uint16_t Timer);
    void onCTCommunicationLost();
    void onNodeReceivedError(uint8_t newErrorFlags);

    // Configuration (0:Socket / 1:Fixed Cable)
    uint8_t config = CONFIG_SOCKET;
    // EVSE mode (0:Normal / 1:Smart / 2:Solar)
    uint8_t mode = MODE_NORMAL;
    uint8_t state = STATE_A_STANDBY;
    uint8_t errorFlags = ERROR_FLAG_NO_ERROR;
    // Residual Current Monitor (0:Disable / 1:Enable)
    uint8_t RCmon = RC_MON_DISABLED;
    // CT calibration value
    uint16_t ICal = ICAL_DEFAULT;
    // Uncalibrated CT1 measurement (resolution 10mA)
    uint16_t Iuncal = 0;
    // Momentary current per Phase (23 = 2.3A) (resolution 100mA). Max 3 phases supported
    int32_t Irms[3] = {0, 0, 0};
    uint16_t solarStopTimer = 0;
    // Temperature EVSE in deg C (-50 to +125)
    int8_t temperature = 0;
    int8_t maxTemperature = DEFAULT_MAX_TEMPERATURE;
    // Calculated Charge Current (Amps *10)
    uint16_t chargeCurrent;
    // External switch (0:Disable / 1:Access B / 2:Access S / 3:Smart-Solar B / 4:Smart-Solar S)
    uint8_t externalSwitch = SWITCH_DISABLED;
    // Cable limit (A) (limited by the wire in the charge cable, set automatically, or manually if Config=Fixed Cable)
    uint16_t cableMaxCapacity;
    // Max Charge current (A)
    uint16_t maxCurrent = MAX_CURRENT;
    // Minimal current the EV is happy with (A)
    uint16_t minCurrent = MIN_CURRENT;
    // Max Mains Amps (hard limit, limited by the MAINS connection) (A)
    uint16_t maxMains = MAX_MAINS;
    uint16_t solarStartCurrent = SOLAR_START_CURRENT;
    uint16_t solarStopTime = SOLAR_STOP_TIME_MINUTES;
    uint16_t solarImportCurrent = SOLAR_IMPORT_CURRENT;
    uint8_t statusConfigChanged = 0;

   protected:
    static void IRAM_ATTR onTimerA();
    static void IRAM_ATTR onCPpulseInterrupt();

   private:
    void readEpromSettings();
    void writeEpromSettings();
    void validateSettings();
    uint16_t IRAM_ATTR local_adc1_read(int channel);
    void stopChargingOnError();
    void sampleADC();
    void sampleTemperatureSensor();
    void sampleExternalSwitch();
    void sampleRCMSensor();
    void sampleProximityPilot();
    void sampleControlPilotLine();
    void onCPpulse();
    void onExternalSwitchInputPulledLow();
    void onExternalSwitchInputReleased();
    void onDiodeOK();
    void statesWorkflow();
    void statesWorkflowStandby();
    void statesWorkflowDetected();
    bool isEnoughPower();
    void onNotEnoughPower();
    void waitForEnoughPower();
    void onPowerBackOn();

    uint8_t controlPilot = 0;
    bool isDiodeOk = false;
    uint8_t externalSwitchReadsCount = 0;
    uint8_t externalSwitchLastValue = 1;
    unsigned long RB2Timer = 0;
    unsigned long temperatureLastSampleMillis = 0;
    // Let car DISCONNECT_WAIT_MILLIS seconds to process electric signal on the fly before disconnecting
    unsigned long disconnectRequestMillis = 0;
    // Turn off current if unused for 30 seconds (CURRENTAVAILABLE_WAIT_TIMEOUT_MILLIS)
    unsigned long currentAvailableWaitMillis = 0;
    // Delays charging at least 60 seconds in case of not enough current available (DEFAULT_CHARGE_DELAY_SECONDS)
    uint8_t chargeDelaySeconds = 0;
    unsigned long chargeDelayLastMillis = 0;
    unsigned long solarStopTimerLastMillis = 0;
    unsigned long chargingNoPwmTimeoutMillis = 0;

    // declared volatile, as they are used in a ISR
    volatile uint16_t ADCsamples[ADC_SAMPLES_SIZE];
    volatile uint16_t adcsample = 0;
    volatile uint8_t adcSamplesIterator = 0;
    esp_adc_cal_characteristics_t* adc_chars_CP;
    esp_adc_cal_characteristics_t* adc_chars_PP;
    esp_adc_cal_characteristics_t* adc_chars_Temperature;

    hw_timer_t* timerA = NULL;
};

extern EVSEController evseController;

#endif