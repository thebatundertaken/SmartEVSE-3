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

#include "EVSEController.h"

#include <Arduino.h>
#include <Preferences.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>

#include "EVSECluster.h"
#include "EVSELogger.h"
#include "EVSEModbus.h"
#include "EVSEPin.h"
#include "EVSERFID.h"
#include "EVSEWifi.h"
#include "EVSEWorkflow.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_adc_cal.h"
#include "soc/rtc_io_struct.h"

// 5% of PWM
#define PWM_5 50
// 95% of PWM
#define PWM_95 950
// 100% of PWM
#define PWM_100 1000

#define MODBUS_BAUDRATE 9600

#define CONTACTOR1_ON digitalWrite(PIN_SSR, HIGH);
#define CONTACTOR1_OFF digitalWrite(PIN_SSR, LOW);

#define CONTACTOR2_ON digitalWrite(PIN_SSR2, HIGH);
#define CONTACTOR2_OFF digitalWrite(PIN_SSR2, LOW);

const char* PREFS_CONTROLLER_NAMESPACE = "settings";
const char* PREFS_RCMON_KEY = "RCmon";
const char* PREFS_MODE_KEY = "Mode";
const char* PREFS_CONFIG_KEY = "Config";
const char* PREFS_SWITCH_KEY = "Switch";
const char* PREFS_MAXCURRENT_KEY = "MaxCurrent";
const char* PREFS_MINCURRENT_KEY = "MinCurrent";
const char* PREFS_MAXMAINS_KEY = "MaxMains";
const char* PREFS_STARTCURRENT_KEY = "StartCurrent";
const char* PREFS_STOPTIME_KEY = "StopTime";
const char* PREFS_IMPORTCURRENT_KEY = "ImportCurrent";
const char* PREFS_MAXTEMP_KEY = "Temperature";
const char* PREFS_SOLARBOOST_KEY = "SolarBost";
const char* PREFS_SOLARBOOST_FACTOR_KEY = "SolarBostF";

// Alarm interrupt handler
// in STATE A this is called every 1ms (autoreload)
// in STATE B/C there is a PWM signal, and the Alarm is set to 5% after the
// low-> high transition of the PWM signal
void IRAM_ATTR EVSEController::onTimerA() {
    evseController.sampleADC();
}

// CP pin low to high transition ISR
void IRAM_ATTR EVSEController::onCPpulseInterrupt() {
    evseController.onCPpulse();
}

void EVSEController::onDiodeCheckOK() {
    // Enable Timer alarm, set to start of CP signal (5%)
    timerAlarmWrite(timerA, PWM_5, false);
}

bool EVSEController::isVehicleConnected() {
    // when access bit = 1, p.ex. in OFF mode, the STATEs are no longer updated
    return controlPilot != CONTROL_PILOT_12V;
}

// Set EVSE mode
void EVSEController::switchMode(uint8_t newMode) {
#if EVSE_FEATFLAG_ENABLE_RFID
    setAccess(true);
#endif

    mode = newMode;
#if EVSE_FEATFLAG_ENABLE_POWERSHARE
    evseCluster.setMasterNodeControllerMode(mode);
#endif

    cleanupNoPowerTimersFlags();
    if (mode != MODE_SMART) {
        solarBoostCurrent = 0;
    }
}

//  Change from Solar to Smart mode and vice versa
void EVSEController::switchModeSolarSmart() {
    switchMode(mode == MODE_SOLAR ? MODE_SMART : MODE_SOLAR);
}

void EVSEController::cleanupNoPowerTimersFlags() {
    // Clear All errors
    errorFlags &= ~(ERROR_FLAG_NO_SUN | ERROR_FLAG_LESS_6A);
    // Clear any Chargedelay
    chargeDelaySeconds = 0;
    // Also make sure the SolarTimer is disabled.
    setSolarStopTimer(0);
}

void EVSEController::onCPpulse() {
    // reset timer, these functions are in IRAM !
    timerWrite(timerA, 0);
    timerAlarmEnable(timerA);
}

// Some low level stuff here to setup the ADC, and perform the conversion.
uint16_t IRAM_ATTR EVSEController::local_adc1_read(int channel) {
    uint16_t adc_value;

    SENS.sar_read_ctrl.sar1_dig_force = 0;                      // switch SARADC into RTC channel
    SENS.sar_meas_wait2.force_xpd_sar = SENS_FORCE_XPD_SAR_PU;  // adc_power_on
    RTCIO.hall_sens.xpd_hall = false;                           // disable other peripherals

    // adc_ll_amp_disable()  // Close ADC AMP module if don't use it for power
    // save.
    SENS.sar_meas_wait2.force_xpd_amp = SENS_FORCE_XPD_AMP_PD;  // channel is set in the convert function
    // disable FSM, it's only used by the LNA.
    SENS.sar_meas_ctrl.amp_rst_fb_fsm = 0;
    SENS.sar_meas_ctrl.amp_short_ref_fsm = 0;
    SENS.sar_meas_ctrl.amp_short_ref_gnd_fsm = 0;
    SENS.sar_meas_wait1.sar_amp_wait1 = 1;
    SENS.sar_meas_wait1.sar_amp_wait2 = 1;
    SENS.sar_meas_wait2.sar_amp_wait3 = 1;

    // adc_hal_set_controller(ADC_NUM_1, ADC_CTRL_RTC);         //Set controller
    // see esp-idf/components/hal/esp32/include/hal/adc_ll.h
    SENS.sar_read_ctrl.sar1_dig_force = 0;       // 1: Select digital control;       0: Select RTC control.
    SENS.sar_meas_start1.meas1_start_force = 1;  // 1: SW control RTC ADC start;     0: ULP control RTC ADC start.
    SENS.sar_meas_start1.sar1_en_pad_force = 1;  // 1: SW control RTC ADC bit map;   0: ULP control RTC ADC bit map;
    SENS.sar_touch_ctrl1.xpd_hall_force = 1;     // 1: SW control HALL power;        0: ULP FSM control HALL power.
    SENS.sar_touch_ctrl1.hall_phase_force = 1;   // 1: SW control HALL phase;        0: ULP FSM control HALL phase.

    // adc_hal_convert(ADC_NUM_1, channel, &adc_value);
    // see esp-idf/components/hal/esp32/include/hal/adc_ll.h
    SENS.sar_meas_start1.sar1_en_pad = (1 << channel);  // select ADC channel to sample on
    while (SENS.sar_slave_addr1.meas_status != 0)
        ;  // wait for conversion to be idle (blocking)
    SENS.sar_meas_start1.meas1_start_sar = 0;
    SENS.sar_meas_start1.meas1_start_sar = 1;  // start ADC conversion
    while (SENS.sar_meas_start1.meas1_done_sar == 0)
        ;                                             // wait (blocking) for conversion to finish
    adc_value = SENS.sar_meas_start1.meas1_data_sar;  // read ADC value from register

    return adc_value;
}

void EVSEController::sampleADC() {
    RTC_ENTER_CRITICAL();
    adcsample = local_adc1_read(ADC1_CHANNEL_3);
    RTC_EXIT_CRITICAL();

    ADCsamples[adcSamplesIterator++] = adcsample;

    if (adcSamplesIterator == ADC_SAMPLES_SIZE) {
        adcSamplesIterator = 0;
    }
}

uint16_t EVSEController::getCableMaxCapacity() {
    // CableMaxCapacity = 0 corner case
    // No cable info (maybe CONFIG_SOCKET and no cable attached)
    return cableMaxCapacity == 0 ? maxDeviceCurrent : cableMaxCapacity;
};

uint16_t EVSEController::getMaxCurrentAvailable() {
    //  Maximum current this EVSE is allow to dispatch (limited by cable and maxmain)
    // Call method getCableMaxCapacity to avoid cableMaxCapacity = 0 corner case
    return _min(getCableMaxCapacity(), maxMains) * 10;
};

int16_t EVSEController::getMainsMeasuredCurrent(bool allowSolarSurplus) {
    int16_t imeasured = 0;

    for (uint8_t x = 0; x < 3; x++) {
        imeasured += Irms[x];
    }

    // Solar surplus might create negative results, only intesting for MODE_SOLAR and/or debugging
    return allowSolarSurplus ? imeasured : _max(imeasured, 0);
}

uint16_t EVSEController::getMainsExtraSolarSurplus() {
    uint16_t extraSurplus = 0;

    for (uint8_t x = 0; x < 3; x++) {
        if (Irms[x] < 0) {
            extraSurplus += abs(Irms[x]);
        }
    }

    return extraSurplus;
}

//  Tell EV to stop charging. When we are not charging switch to State B1
void EVSEController::stopChargingOnError() {
    EVSELogger::debug("[EVSEController] Stopping charging due to charge error");

    switch (state) {
        case STATE_C_CHARGING:
            setState(STATE_C1_CHARGING_NO_POWER);
            break;

        case STATE_B_VEHICLE_DETECTED:
            setState(STATE_B1_VEHICLE_DETECTED_NO_POWER);
            break;

        default:
            // Invalid state
            return;
    }

    evseCluster.setMasterNodeBalancedState(state);
}

void EVSEController::activateChargeDelay() {
    //  Set Chargedelay when no PV meter and/or MODE_NORMAL
    if ((mode == MODE_NORMAL || evseModbus.mainsMeter == MAINS_METER_DISABLED) && chargeDelaySeconds == 0) {
        chargeDelaySeconds = DEFAULT_CHARGE_DELAY_SECONDS;
    }
}

void EVSEController::onNodeReceivedError(uint8_t newErrorFlags) {
    errorFlags = newErrorFlags;

    if (errorFlags) {
        EVSELogger::info("[EVSEController] Node received error. Stopping charging");
        stopChargingOnError();
        activateChargeDelay();
    }
}

#if EVSE_FEATFLAG_ENABLE_RFID
void EVSEController::setAccess(bool access) {
    evseRFID.rfidAccessBit = access ? RFID_ACCESS_GRANTED : RFID_NO_ACCESS;

    if (!access) {
        EVSELogger::info("[EVSEController] Disabled access bit. Stopping charging");
        stopChargingOnError();
    }
}
#endif

void EVSEController::onCTCommunicationLost() {
    if (errorFlags & ERROR_FLAG_CT_NOCOMM) {
        // Do not re-run if comms are already down
        return;
    }

    errorFlags |= ERROR_FLAG_CT_NOCOMM;
    EVSELogger::info("[EVSEController] CT communication lost. Stopping charging");
    stopChargingOnError();

#if EVSE_FEATFLAG_ENABLE_POWERSHARE
    evseCluster.setMasterNodeErrorflags(errorFlags);
#endif
    evseCluster.resetBalancedStates();
}

void EVSEController::waitForEnoughPower() {
    EVSELogger::info("[EVSEController] Waiting for enough power. Stopping charging");
    stopChargingOnError();
    activateChargeDelay();
}

bool EVSEController::isEnoughPower() {
    // Check flags no_sun and less_6a are off
    return !(errorFlags & (ERROR_FLAG_NO_SUN | ERROR_FLAG_LESS_6A));
}

// Not enough power
void EVSEController::onNotEnoughPower() {
    errorFlags |= (mode == MODE_SOLAR) ? ERROR_FLAG_NO_SUN : ERROR_FLAG_LESS_6A;
}

// MODE_SOLAR low current
void EVSEController::onSolarLowPower() {
    if (state == STATE_A_STANDBY || solarStopTimeMinutes == 0) {
        onSolarStopTimer();
        return;
    }

    if (solarStopTimer == 0 && solarStopTimeMinutes != 0) {
        // Convert minutes into seconds
        setSolarStopTimer(solarStopTimeMinutes * 60);
    }
}

void EVSEController::onSolarChargingEnoughPower() {
    if (solarStopTimer != 0) {
        setSolarStopTimer(0);
    }
}

void EVSEController::onPowerBackOn() {
    errorFlags &= ~ERROR_FLAG_LESS_6A;
    errorFlags &= ~ERROR_FLAG_NO_SUN;
    cleanupNoPowerTimersFlags();

#if EVSE_FEATFLAG_ENABLE_POWERSHARE
    evseCluster.setMasterNodeErrorflags(errorFlags);
#endif
}

void EVSEController::onSolarStopTimer() {
    errorFlags |= ERROR_FLAG_NO_SUN;
    EVSELogger::debug("[EVSEController] Solar mode no sun. Stopping charging");
    stopChargingOnError();

    // reset all states
    evseCluster.resetBalancedStates();
}

void EVSEController::forceDisconnect() {
    onDisconnectInProgress();
    setState(STATE_DISCONNECT_IN_PROGRESS);
}

bool EVSEController::forceStartCharging() {
    if (evseController.state == STATE_C_CHARGING) {
        return false;
    }

    evseController.onVehicleStartCharging();
    evseController.setState(STATE_C_CHARGING);
    return true;
}

void EVSEController::onDisconnectInProgress() {
    // PWM off, channel 0, duty cycle 0%
    ledcWrite(CP_CHANNEL, CONTROL_PILOT_DUTYCICLE_0);
}

void EVSEController::onStandbyReadyToCharge() {
    // Cable corner case: vehicle finish charging, owner remove cable from wallbox (EVSE is a socket)
    if (config == CONFIG_SOCKET) {
        // Reset cableMaxCapacity after vehicle finished charging (cableMaxCapacity holds
        // previous cable max capacity, but that cable is now gone)
        cableMaxCapacity = 0;
    }

    // Waiting for power corner case
    // EVSE still waiting for power, but vehicle is not connected anymore
    // reset flags and timers
    cleanupNoPowerTimersFlags();
}

void EVSEController::onVehicleConnected() {
    sampleProximityPilot();
    resetChargeCurrent();
}

void EVSEController::onVehicleStartCharging() {
    // Use chargeCurrent value instead maxMains to allow current override
    evseCluster.setMasterNodeBalancedMax(chargeCurrent);

    if (!evseCluster.isEnoughCurrentAvailableForAnotherEVSE()) {
        EVSELogger::warn("[EVSEController] Vehicle wants to charge but not enough power");
        onNotEnoughPower();
        return;
    }

    openElectricCircuit();
    // Rebalance chargeCurrent AFTERWARDS
    evseCluster.adjustChargeCurrent();
}

void EVSEController::openElectricCircuit() {
    // Do not open circuit if no power
    if (chargeCurrent <= 0 || !isEnoughPower()) {
        return;
    }

    // Do not open circuit if controller is not in the right state (ex: standby
    // smart mode rebalance)
    if (state == STATE_C_CHARGING || state == STATE_C1_CHARGING_NO_POWER || state == STATE_B_VEHICLE_DETECTED ||
        state == STATE_B1_VEHICLE_DETECTED_NO_POWER) {
        setCurrent(chargeCurrent);
    }
}

void EVSEController::onChargeCurrentChanged() {
    if (chargeCurrent < (minEVCurrent * 10)) {
        sprintf(sprintfStr, "[EVSEController] onChargeCurrentChanged (%d) triggering not enough power", chargeCurrent);
        EVSELogger::warn(sprintfStr);
        onNotEnoughPower();
        return;
    }

    uint16_t maxCurrentAvailable = getMaxCurrentAvailable();
    if (solarBoost && solarBoostCurrent > 0) {
        maxCurrentAvailable += solarBoostCurrent;
    }
    sprintf(sprintfStr, "[EVSEController] [solarBoost] maxCurrentAvailable=%d", maxCurrentAvailable);
    EVSELogger::debug(sprintfStr);

    if (chargeCurrent > maxCurrentAvailable) {
        sprintf(sprintfStr, "[EVSEController] chargeCurrent (%d) above maxCurrentAvailable (%d)", chargeCurrent,
                maxCurrentAvailable);
        EVSELogger::warn(sprintfStr);
        chargeCurrent = maxCurrentAvailable;
    }

    openElectricCircuit();
}

void EVSEController::mainsMeterReadings(int32_t L1, int32_t L2, int32_t L3) {
    if (evseCluster.amIMasterOrLBDisabled()) {
        // Clear communication error, if present
        if (errorFlags & ERROR_FLAG_CT_NOCOMM) {
            errorFlags &= ~ERROR_FLAG_CT_NOCOMM;
        }
        lastMainsMeterResponseMillis = millis();
    }

    // TODO SCF integrate with power meter in EM_API
    //  reduce resolution of Irms to 100mA
    Irms[0] = (signed int)(L1 / 100);
    Irms[1] = (signed int)(L2 / 100);
    Irms[2] = (signed int)(L3 / 100);
}

long EVSEController::getLastMainsMeterResponse() {
    return lastMainsMeterResponseMillis == 0 ? -1 : millis() - lastMainsMeterResponseMillis;
};

// Set Charge Current
// Current in Amps * 10 (160 = 16A)
void EVSEController::setCurrent(uint16_t current) {
    // The “duty cycle” (the length of the pulse) determines the maximum current EVSE can supply to the vehicle
    uint32_t dutyCycle;

    /*Duty cycle < 3 %  	No charging allowed
    3 % ≤ duty cycle ≤ 7 % 	Force high-level communication protocol according to ISO 15118 or DIN 70121
    7 % < duty cycle< 8 % 	No charging allowed
    8 % ≤ duty cycle< 10 % 	Max. current consumption for AC charging is 6 A
    10 % ≤ duty cycle ≤ 85 % 	Available current = duty cycle * 0.6 A
    85 % < duty cycle ≤ 96 % 	Available current = (duty cycle - 64) * 2.5 A
    96 % < duty cycle ≤ 97 % 	Max. current consumption for AC charging is 80 A
    Duty cycle > 97 % 	No charging allowed*/

    if ((current >= 60) && (current <= 510)) {
        dutyCycle = current / 0.6;
        // calculate dutyCycle from current
    } else if ((current > 510) && (current <= MAX_MAINS_HARD_LIMIT)) {
        dutyCycle = (current / 2.5) + 640;
    } else {
        // invalid, use 6A (100 * 0.6 = 60 -> 6A)
        dutyCycle = 100;
    }

    // conversion to 1024 = 100%
    dutyCycle = (dutyCycle * CONTROL_PILOT_DUTYCICLE_100) / 1000;
    // update PWM signal
    ledcWrite(CP_CHANNEL, dutyCycle);
}

void EVSEController::setChargeCurrent(uint16_t value) {
    // hard limit 80A
    if (value > MAX_MAINS_HARD_LIMIT) {
        value = MAX_MAINS_HARD_LIMIT;
    }

    if (chargeCurrent != value) {
        chargeCurrent = value;
        onChargeCurrentChanged();
    }
};

void EVSEController::disableOperatingHours() {
    switchOnTime = 0;
    switchOffTime = 0;
}

bool EVSEController::isOperatingHoursEnabled() {
    return switchOnTime != switchOffTime;
}

bool EVSEController::isChargingInOperatingHours() {
    if (!isOperatingHoursEnabled()) {
        return true;
    }

    uint16_t localTime = evseWifi.getNTPLocalTime();
    if (localTime == UINT16_MAX) {
        return false;
    }

    if (switchOnTime < switchOffTime) {
        return (localTime >= switchOnTime) && (localTime < switchOffTime);
    }

    return (localTime >= switchOnTime) || (localTime < switchOffTime);
}

void EVSEController::setOperatingHours(uint16_t onTime, uint16_t offTime) {
    if (onTime >= 0 && onTime <= 2359 && offTime >= 0 && offTime <= 2359 && onTime != offTime) {
        switchOnTime = onTime;
        switchOffTime = offTime;
    } else {
        disableOperatingHours();
    }
}

void EVSEController::setState(uint8_t NewState) {
    state = NewState;

    switch (state) {
        case STATE_B1_VEHICLE_DETECTED_NO_POWER:
            // When entering State B1, wait at least 3 seconds before switching to
            // another state.
            if (chargeDelaySeconds == 0) {
                chargeDelaySeconds = 3;
            }
            break;

        case STATE_A_STANDBY:
            CONTACTOR1_OFF;
            CONTACTOR2_OFF;
            // PWM off, channel 0, duty cycle 100%
            ledcWrite(CP_CHANNEL, CONTROL_PILOT_DUTYCICLE_100);
            // Alarm every 1ms, auto reload
            timerAlarmWrite(timerA, PWM_100, true);

            cleanupNoPowerTimersFlags();
#if EVSE_FEATFLAG_ENABLE_EVMETER
            evseModbus.evMeterResetKwhOnStandby();
#endif
            chargeDelaySeconds = 0;
            resetChargeCurrent();
            break;

        case STATE_B_VEHICLE_DETECTED:
            evseCluster.setMasterNodeBalancedMax(getMaxCurrentAvailable());
            evseCluster.setMasterNodeBalancedCurrent(chargeCurrent);

            CONTACTOR1_OFF;
            CONTACTOR2_OFF;
            // Enable Timer alarm, set to diode test (95%)
            timerAlarmWrite(timerA, PWM_95, false);
            // Enable PWM
            openElectricCircuit();
            break;

        case STATE_C_CHARGING:
#if EVSE_FEATFLAG_ENABLE_EVMETER
            evseModbus.evMeterResetKwhOnCharging();
#endif
            CONTACTOR1_ON;
            CONTACTOR2_ON;
            break;

        case STATE_C1_CHARGING_NO_POWER:
            // PWM off, channel 0, duty cycle 100%
            ledcWrite(CP_CHANNEL, CONTROL_PILOT_DUTYCICLE_100);
            // Alarm every 1ms, auto reload. EV should detect and stop charging within 3 seconds
            timerAlarmWrite(timerA, PWM_100, true);
            // Keep in State C1 for 15 seconds, so the charge cable can be removed
            chargeDelaySeconds = STATE_C1_CHARGING_NO_POWER_DELAY_SECONDS;
            break;

        case STATE_MODBUS_COMM_C:
#if EVSE_FEATFLAG_ENABLE_EVMETER
            evseModbus.evMeterResetKwhOnCharging();
#endif
            break;
    }

    evseCluster.setMasterNodeBalancedState(state);
}

// Sample the Temperature sensor
void EVSEController::sampleTemperatureSensor() {
    temperatureLastSampleMillis = millis();

    RTC_ENTER_CRITICAL();
    uint32_t sample = local_adc1_read(ADC1_CHANNEL_0);
    RTC_EXIT_CRITICAL();

    // voltage range is from 0-2200mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(sample, adc_chars_Temperature);

    // The MCP9700A temperature sensor outputs 500mV at 0C, and has a 10mV/C
    // change in output voltage. so 750mV is 25C, 400mV = -10C
    temperature = (signed int)(voltage - 500) / 10;
    sprintf(sprintfStr, "[EVSEController] Temp: %i C (%u mV)", temperature, voltage);
    EVSELogger::debug(sprintfStr);

    // Temperature between limit?
    if (temperature < (maxTemperature - TEMPERATURE_COOLDOWN_THRESHOLD)) {
        // Clear Error
        if (errorFlags & ERROR_FLAG_TEMP_HIGH) {
            errorFlags &= ~ERROR_FLAG_TEMP_HIGH;
        }

        return;
    }

    // Temperature too High?
    if (temperature >= maxTemperature && !(errorFlags & ERROR_FLAG_TEMP_HIGH)) {
        errorFlags |= ERROR_FLAG_TEMP_HIGH;
        // ERROR, switch back to STATE_A_STANDBY
        setState(STATE_A_STANDBY);
        evseCluster.resetBalancedStates();
    }
}

// The CP (control pilot) output is a fixed 1khz square-wave (+6..9v / -12v).
// It's pulse width varies between 10% and 96% indicating 6A-80A charging current.
// to detect state changes we should measure the CP signal while it's at ~5% (so 50uS after the positive pulse started)
// we use an i/o interrupt at the CP pin output, and a one shot timer interrupt to start the ADC conversion.
// would be nice if there was an easier way...

// Determine the state of the Pilot signal
void EVSEController::sampleControlPilotLine() {
    uint32_t sample;
    uint32_t min = 3300;
    uint32_t max = 0;
    uint32_t voltage;

    // make sure we wait 100ms after each state change before calculating Average
    // if ( (StateTimer + 100) > millis() ) return CONTROL_PILOT_WAIT;

    // calculate Min/Max of last 25 CP measurements
    for (uint8_t n = 0; n < ADC_SAMPLES_SIZE; n++) {
        sample = ADCsamples[n];
        // convert adc reading to voltage
        voltage = esp_adc_cal_raw_to_voltage(sample, adc_chars_CP);
        // store lowest and highest values
        if (voltage < min) {
            min = voltage;
        }

        if (voltage > max) {
            max = voltage;
        }
    }

    if (prevMin != min) {
        prevMin = min;
        sprintf(sprintfStr, "[EVSEController] CP min changed from %u to %u", prevMin, min);
        EVSELogger::debug(sprintfStr);
    }
    if (prevMax != max) {
        prevMax = max;
        sprintf(sprintfStr, "[EVSEController] CP max changed from %u to %u", prevMax, max);
        EVSELogger::debug(sprintfStr);
    }

    /*
    +12 V 	State A 	No EV connected to the EVSE
    +9 V 	State B 	EV connected to the EVSE, but not ready for charging
    +6 V 	State C 	Connected and ready for charging, ventilation is not required
    +3 V 	State D 	Connected, ready for charging and ventilation is required
    +0 V 	State E 	Electrical short to earth on the controller of the EVSE, no power supply
    -12 V 	State F 	EVSE is unavailable
    */

    // test Min/Max against fixed levels
    if (min > 3000) {
        // Pilot at 12V (min 11.0V)
        controlPilot = CONTROL_PILOT_12V;
        return;
    }

    if ((min > 2700) && (max < 2930)) {
        controlPilot = CONTROL_PILOT_9V;
        return;
    }

    if ((min > 2400) && (max < 2600)) {
        controlPilot = CONTROL_PILOT_6V;
        return;
    }

    if ((min > 100) && (max < 300)) {
        controlPilot = CONTROL_PILOT_DIODE_CHECK_OK;
        return;
    }

    controlPilot = CONTROL_PILOT_NOK;
}

void EVSEController::setSolarStopTimer(uint16_t timer) {
    if (solarStopTimer == timer) {
        return;
    }

    solarStopTimer = timer;
#if EVSE_FEATFLAG_ENABLE_POWERSHARE
    evseCluster.setMasterSolarStopTimer(solarStopTimer);
#endif
}

void EVSEController::setSolarBoost(bool active) {
    solarBoost = active;
};

int16_t EVSEController::calcSolarBoostCurrent() {
    // Is solar boost on?
    if (!solarBoost || mode != MODE_SMART) {
        solarBoostCurrent = 0;
        return solarBoostCurrent;
    }

    int16_t extraSolarSurplus = getMainsExtraSolarSurplus();
    sprintf(sprintfStr, "[EVSEController] [solarBoost] extraSolarSurplus=%d", extraSolarSurplus);
    EVSELogger::debug(sprintfStr);

    // Is there any solar surplus?
    if (extraSolarSurplus <= 0) {
        solarBoostCurrent = 0;
        return solarBoostCurrent;
    }

    if (state == STATE_C_CHARGING) {
        // EV charging but there are surplus. It happens on 3 phases charges with 1 phase cable
        uint16_t maxCurrentCache = getMaxCurrentAvailable();
        extraSolarSurplus = maxCurrentCache - getMainsMeasuredCurrent(true);
        int16_t chargeCurrentDiff = getChargeCurrent() - maxCurrentCache;
        if (chargeCurrentDiff < 0) {
            // difference < 0: Exceeded power, immediately decrease current to match maximum available
            solarBoostCurrent = _min(extraSolarSurplus, 0);
            sprintf(
                sprintfStr,
                "[EVSEController] [solarBoost] Charging exceeded power! solarBoostCurrent = %d; chargeCurrentDiff = %d",
                solarBoostCurrent, chargeCurrentDiff);
        } else if (chargeCurrentDiff > 0) {
            // difference > 0: Spare power, slowly increase current by % of difference
            float extraPower = ceil((float)chargeCurrentDiff * solarBoostIncreaseFactor);
            solarBoostCurrent = _min(extraSolarSurplus + extraPower, maxCurrentCache);
            sprintf(sprintfStr,
                    "[EVSEController] [solarBoost] Charging with spare solar power. maxCurrent = %.1fA; solarSurplus = "
                    "%.1fA; spareDiff = %.1fA; extra power = %.1fA; factor = %.2f; new solarBoostCurrent = %.1fA",
                    ((float)maxCurrentCache / 10), ((float)extraSolarSurplus / 10), ((float)chargeCurrentDiff / 10),
                    (extraPower / 10), solarBoostIncreaseFactor, ((float)solarBoostCurrent / 10));
        } else {
            // diference == 0: Perfectly balanced
            solarBoostCurrent = extraSolarSurplus;
            sprintf(sprintfStr, "[EVSEController] [solarBoost] Charging perfectly balanced. solarBoostCurrent = %d",
                    solarBoostCurrent);
        }

        EVSELogger::debug(sprintfStr);
    } else {
        solarBoostCurrent = extraSolarSurplus;
        sprintf(sprintfStr, "[EVSEController] [solarBoost] solarBoostCurrent=%d", solarBoostCurrent);
        EVSELogger::debug(sprintfStr);
    }

    return solarBoostCurrent;
}

#if EVSE_FEATFLAG_ENABLE_RCMON
// RCM = Residual Current Monitor
// Clear RCM error bit, by pressing any button
void EVSEController::resetRCMErrorFlag() {
    // RCM was tripped, but RCM level is back to normal. Clear RCM error bit, by
    // pressing any button
    if (RCmon == RC_MON_ENABLED && (errorFlags & ERROR_FLAG_RCM_TRIPPED) && digitalRead(PIN_RCM_FAULT) == LOW) {
        errorFlags &= ~ERROR_FLAG_RCM_TRIPPED;
    }
}

void EVSEController::sampleRCMSensor() {
    if (RCmon == RC_MON_DISABLED) {
        return;
    }

    // Residual current monitor active and DC current > 6mA ?
    if (digitalRead(PIN_RCM_FAULT) == HIGH) {
        delay(1);
        // check again, to prevent voltage spikes from tripping the RCM detection
        if (digitalRead(PIN_RCM_FAULT) == HIGH) {
            if (state != STATE_A_STANDBY) {
                setState(STATE_B1_VEHICLE_DETECTED_NO_POWER);
            }
            errorFlags = ERROR_FLAG_RCM_TRIPPED;
        }
    }
}
#endif

#if EVSE_FEATFLAG_ENABLE_EXTSWITCH
void EVSEController::onExternalSwitchInputPulledLow() {
    switch (externalSwitch) {
        case SWITCH_DISABLED:
            if (state == STATE_C_CHARGING) {
                setState(STATE_C1_CHARGING_NO_POWER);
            }
            break;

        case SWITCH_ACCESS_BUTTON:
#if EVSE_FEATFLAG_ENABLE_RFID
            // Toggle Access bit on/off
            setAccess(!evseRFID.rfidAccessBit);
#endif
            break;

        case SWITCH_ACCESS_SWITCH:
#if EVSE_FEATFLAG_ENABLE_RFID
            setAccess(true);
#endif
            break;

        case SWITCH_SMARTSOLAR_BUTTON:
            // Smart-Solar Button or hold button for 1,5 second to STOP charging
            if (RB2Timer == 0) {
                RB2Timer = millis();
            }

            if (RB2Timer != 0 && ((millis() - RB2Timer) > SWITCH_SMARTSOLAR_BUTTON_LONGPRESSED_MILLIS)) {
                if (state == STATE_C_CHARGING) {
                    setState(STATE_C1_CHARGING_NO_POWER);
                }
            }
            break;

        case SWITCH_SMARTSOLAR_SWITCH:
            if (mode == MODE_SOLAR) {
                switchModeSolarSmart();
            }
            break;
    }

#if EVSE_FEATFLAG_ENABLE_RCMON
    // Reset RCM error when button is pressed. RCM was tripped, but RCM level is
    // back to normal
    resetRCMErrorFlag();
#endif
}

void EVSEController::onExternalSwitchInputReleased() {
    switch (externalSwitch) {
        case SWITCH_ACCESS_SWITCH:
#if EVSE_FEATFLAG_ENABLE_RFID
            setAccess(false);
#endif
            break;

        case SWITCH_SMARTSOLAR_BUTTON:
            if (RB2Timer != 0 && ((millis() - RB2Timer) > SWITCH_SMARTSOLAR_BUTTON_LONGPRESSED_MILLIS)) {
                switchModeSolarSmart();
            }
            RB2Timer = 0;
            break;

        case SWITCH_SMARTSOLAR_SWITCH:
            if (mode == MODE_SMART) {
                switchModeSolarSmart();
            }
            break;
    }
}

// Check the external switch
void EVSEController::sampleExternalSwitch() {
    // RB2Timer != 0 means switch is being hold by user and not released
    if (RB2Timer == 0) {
        if (digitalRead(PIN_SW_IN) == externalSwitchLastValue) {
            // External switch state hasn't changed
            externalSwitchReadsCount = 0;
            return;
        }

        // Make sure that noise on the input does not switch (at least 5 times)
        if (externalSwitchReadsCount++ <= 5) {
            return;
        }
    }

    externalSwitchReadsCount = 0;
    externalSwitchLastValue = digitalRead(PIN_SW_IN);
    if (externalSwitchLastValue == 0) {
        EVSELogger::info("[EVSEController] sampleExternalSwitch triggered pulled low");
        onExternalSwitchInputPulledLow();
    } else {
        EVSELogger::info("[EVSEController] sampleExternalSwitch triggered input released");
        onExternalSwitchInputReleased();
    }
}
#endif

void EVSEController::resetChargeCurrent() {
    //  Do not exceed cable max capacity
    setChargeCurrent(getMaxCurrentAvailable());
    solarBoostCurrent = 0;
}

// Sample the Proximity Pin, and determine the maximum current the cable can
// handle.
void EVSEController::sampleProximityPilot() {
    // PP will be sample only for CONFIG_SOCKET mode, since PP wire is not in use for CONFIG_FIXED_CABLE
    if (config == CONFIG_FIXED_CABLE) {
        cableMaxCapacity = maxDeviceCurrent;
        return;
    }

    RTC_ENTER_CRITICAL();
    uint32_t sample = local_adc1_read(ADC1_CHANNEL_6);
    RTC_EXIT_CRITICAL();

    uint32_t voltage = esp_adc_cal_raw_to_voltage(sample, adc_chars_PP);
    sprintf(sprintfStr, "[EVSEController] PP pin: %u (%u mV)\n", sample, voltage);
    EVSELogger::debug(sprintfStr);

    if ((voltage > 1200) && (voltage < 1400)) {
        // Max cable current = 16A	680R -> should be around 1.3V
        cableMaxCapacity = 16;
        return;
    }

    if ((voltage > 500) && (voltage < 700)) {
        // Max cable current = 32A	220R -> should be around 0.6V
        cableMaxCapacity = 32;
        return;
    }

    if ((voltage > 200) && (voltage < 400)) {
        // Max cable current = 63A	100R -> should be around 0.3V
        cableMaxCapacity = 63;
        return;
    }

    // No resistor, set capacity to device max
    cableMaxCapacity = maxDeviceCurrent;
}

void EVSEController::updateSettings() {
    errorFlags = ERROR_FLAG_NO_ERROR;
    chargeDelaySeconds = 0;
    setSolarStopTimer(0);

    writeEpromSettings();
}

void EVSEController::resetSettings() {
    readEpromSettings();
}

void EVSEController::readEpromSettings() {
    Preferences preferences;
    if (preferences.begin(PREFS_CONTROLLER_NAMESPACE, true) != true) {
        EVSELogger::error("Unable to open preferences for EVSEController");
        return;
    }

    bool firstRun = true;
    if (preferences.isKey(PREFS_RCMON_KEY)) {
        firstRun = false;

#if EVSE_FEATFLAG_ENABLE_RCMON
        RCmon = preferences.getUChar(PREFS_RCMON_KEY, RC_MON_DISABLED);
#endif
        mode = preferences.getUChar(PREFS_MODE_KEY, MODE_NORMAL);
        config = preferences.getUChar(PREFS_CONFIG_KEY, CONFIG_SOCKET);
#if EVSE_FEATFLAG_ENABLE_EXTSWITCH
        externalSwitch = preferences.getUChar(PREFS_SWITCH_KEY, SWITCH_DISABLED);
#endif
        maxDeviceCurrent = preferences.getUShort(PREFS_MAXCURRENT_KEY, MAX_DEVICE_CURRENT);
        minEVCurrent = preferences.getUShort(PREFS_MINCURRENT_KEY, MIN_EV_CURRENT);
        maxMains = preferences.getUShort(PREFS_MAXMAINS_KEY, MAX_MAINS);
        solarStartCurrent = preferences.getUShort(PREFS_STARTCURRENT_KEY, SOLAR_START_CURRENT);
        solarStopTimeMinutes = preferences.getUShort(PREFS_STOPTIME_KEY, SOLAR_STOP_TIME_MINUTES);
        solarImportCurrent = preferences.getUShort(PREFS_IMPORTCURRENT_KEY, SOLAR_IMPORT_CURRENT);
        maxTemperature = preferences.getChar(PREFS_MAXTEMP_KEY, DEFAULT_MAX_TEMPERATURE);
        solarBoost = preferences.getChar(PREFS_SOLARBOOST_KEY, SOLAR_BOOST_DISABLED);
        solarBoostIncreaseFactor = preferences.getChar(PREFS_SOLARBOOST_FACTOR_KEY, DEFAULT_SOLARBOOST_INCREASE_FACTOR);
    }
    preferences.end();

    if (firstRun) {
#if EVSE_FEATFLAG_ENABLE_RCMON
        RCmon = RC_MON_DISABLED;
#endif
        mode = MODE_NORMAL;
        config = CONFIG_SOCKET;
#if EVSE_FEATFLAG_ENABLE_EXTSWITCH
        externalSwitch = SWITCH_DISABLED;
#endif
        maxDeviceCurrent = MAX_DEVICE_CURRENT;
        minEVCurrent = MIN_EV_CURRENT;
        maxMains = MAX_MAINS;
        solarStartCurrent = SOLAR_START_CURRENT;
        solarStopTimeMinutes = SOLAR_STOP_TIME_MINUTES;
        solarImportCurrent = SOLAR_IMPORT_CURRENT;
        maxTemperature = DEFAULT_MAX_TEMPERATURE;
        solarBoost = SOLAR_BOOST_DISABLED;
        solarBoostIncreaseFactor = DEFAULT_SOLARBOOST_INCREASE_FACTOR;
        writeEpromSettings();
    }

    // We might need some sort of authentication in the future.
    // SmartEVSE v3 have programmed ECDSA-256 keys stored in nvs
    // Unused for now.
    /*if (preferences.begin("KeyStorage", true) == true) {
        serialnr = preferences.getUInt("serialnr");
        // 0x0101 (01 = SmartEVSE,  01 = hwver 01)
        // uint16_t hwversion = preferences.getUShort("hwversion");
        // String ec_private = preferences.getString("ec_private");
        // String ec_public = preferences.getString("ec_public");
        preferences.end();

        // overwrite APhostname if serialnr is programmed
        // SmartEVSE access point Name = SmartEVSE-xxxxx
        //APhostname = "SmartEVSE-" + String(serialnr & 0xffff, 10);
        // EVSELogger.debug("hwversion %04x serialnr:%u \n",hwversion, serialnr);
        // EVSELogger.debug(ec_public);
    } else {
        EVSELogger.error(("[EVSEController] No KeyStorage found in nvs!\n");
    }*/
}

void EVSEController::writeEpromSettings() {
    Preferences preferences;
    if (preferences.begin(PREFS_CONTROLLER_NAMESPACE, false) != true) {
        EVSELogger::error("Unable to write preferences for EVSEController");
        return;
    }

#if EVSE_FEATFLAG_ENABLE_RCMON
    preferences.putUChar(PREFS_RCMON_KEY, RCmon);
#endif
    preferences.putUChar(PREFS_MODE_KEY, mode);
    preferences.putUChar(PREFS_CONFIG_KEY, config);
#if EVSE_FEATFLAG_ENABLE_EXTSWITCH
    preferences.putUChar(PREFS_SWITCH_KEY, externalSwitch);
#endif
    preferences.putUShort(PREFS_MAXCURRENT_KEY, maxDeviceCurrent);
    preferences.putUShort(PREFS_MINCURRENT_KEY, minEVCurrent);
    preferences.putUShort(PREFS_MAXMAINS_KEY, maxMains);
    preferences.putUShort(PREFS_STARTCURRENT_KEY, solarStartCurrent);
    preferences.putUShort(PREFS_STOPTIME_KEY, solarStopTimeMinutes);
    preferences.putUShort(PREFS_IMPORTCURRENT_KEY, solarImportCurrent);
    preferences.putChar(PREFS_MAXTEMP_KEY, maxTemperature);
    preferences.putChar(PREFS_SOLARBOOST_KEY, solarBoost);
    preferences.putChar(PREFS_SOLARBOOST_FACTOR_KEY, solarBoostIncreaseFactor);

    preferences.end();
}

void EVSEController::setup() {
    pinMode(PIN_CP_OUT, OUTPUT);  // CP output
    pinMode(PIN_SW_IN, INPUT);    // SW Switch input
    pinMode(PIN_SSR, OUTPUT);     // SSR1 output
    pinMode(PIN_SSR2, OUTPUT);    // SSR2 output
    pinMode(PIN_RCM_FAULT, INPUT_PULLUP);

    pinMode(PIN_LCD_LED, OUTPUT);  // LCD backlight
    pinMode(PIN_LCD_RST, OUTPUT);  // LCD reset

    // Fix screen goes blank if pins are not fully define before starting up the
    // device < button
    pinMode(PIN_IO0_B1, INPUT);
    // o Select button + A0 LCD
    pinMode(PIN_LCD_A0_B2, OUTPUT);
    // > button + SDA/MOSI pin
    pinMode(PIN_LCD_SDO_B3, OUTPUT);

    pinMode(PIN_LOCK_IN, INPUT);      // Locking Solenoid input
    pinMode(PIN_LEDR, OUTPUT);        // Red LED output
    pinMode(PIN_LEDG, OUTPUT);        // Green LED output
    pinMode(PIN_LEDB, OUTPUT);        // Blue LED output
    pinMode(PIN_ACTUATOR_A, OUTPUT);  // Actuator Driver output R
    pinMode(PIN_ACTUATOR_B, OUTPUT);  // Actuator Driver output W
    pinMode(PIN_CPOFF, OUTPUT);       // Disable CP output (active high)
    pinMode(PIN_RS485_RX, INPUT);
    pinMode(PIN_RS485_TX, OUTPUT);
    pinMode(PIN_RS485_DIR, OUTPUT);

    digitalWrite(PIN_LEDR, LOW);
    digitalWrite(PIN_LEDG, LOW);
    digitalWrite(PIN_LEDB, LOW);
    digitalWrite(PIN_ACTUATOR_A, LOW);
    digitalWrite(PIN_ACTUATOR_B, LOW);
    digitalWrite(PIN_CPOFF, LOW);     // CP signal ACTIVE
    digitalWrite(PIN_SSR, LOW);       // SSR1 OFF
    digitalWrite(PIN_SSR2, LOW);      // SSR2 OFF
    digitalWrite(PIN_LCD_LED, HIGH);  // LCD Backlight ON

    // Uart 0 debug/program port
    Serial.begin(115200);
    while (!Serial)
        ;
    EVSELogger::info("[EVSEController] SmartEVSE powerup");

    // configure SPI connection to LCD
    // only the SPI_SCK and SPI_MOSI pins are used
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_SS);
    // the ST7567's max SPI Clock frequency is 20Mhz at 3.3V/25C
    // We choose 10Mhz here, to reserve some room for error.
    // SPI mode is MODE3 (Idle = HIGH, clock in on rising edge)
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3));

    // setup timer, and one shot timer interrupt to 50us
    timerA = timerBegin(0, 80, true);
    timerAttachInterrupt(timerA, &onTimerA, false);
    // we start in STATE A, with a static +12V CP signal
    // set alarm to trigger every 1mS, and let it reload every 1ms
    timerAlarmWrite(timerA, PWM_100, true);
    // when PWM is active, we sample the CP pin after 5%
    timerAlarmEnable(timerA);

    // Setup ADC on CP, PP and Temperature pin
    adc1_config_width(ADC_WIDTH_BIT_10);  // 10 bits ADC resolution is enough
    adc1_config_channel_atten(ADC1_CHANNEL_3,
                              ADC_ATTEN_DB_12);  // setup the CP pin input attenuation to 11db
    adc1_config_channel_atten(ADC1_CHANNEL_6,
                              ADC_ATTEN_DB_6);  // setup the PP pin input attenuation to 6db
    adc1_config_channel_atten(ADC1_CHANNEL_0,
                              ADC_ATTEN_DB_6);  // setup the Temperature input attenuation to 6db

    // Characterize the ADC at particular attentuation for each channel
    adc_chars_CP = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    adc_chars_PP = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    adc_chars_Temperature = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type =
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_10, 1100, adc_chars_CP);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_10, 1100, adc_chars_PP);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_10, 1100, adc_chars_Temperature);

    // Setup PWM on channel 0, 1000Hz, 10 bits resolution
    ledcSetup(CP_CHANNEL, 1000, 10);  // channel 0 => Group: 0, Channel: 0, Timer: 0
    // setup the RGB led PWM channels
    // as PWM channel 1 is used by the same timer as the CP timer (channel 0), start with channel 2
    ledcSetup(RED_CHANNEL, 5000, 8);    // R channel 2, 5kHz, 8 bit
    ledcSetup(GREEN_CHANNEL, 5000, 8);  // G channel 3, 5kHz, 8 bit
    ledcSetup(BLUE_CHANNEL, 5000, 8);   // B channel 4, 5kHz, 8 bit

    // attach the channels to the GPIO to be controlled
    ledcAttachPin(PIN_CP_OUT, CP_CHANNEL);
    // pinMode(PIN_CP_OUT, OUTPUT);                // Re-init the pin to output,
    // required in order for attachInterrupt to work (2.0.2)
    //  not required/working on master branch..
    //  see https://github.com/espressif/arduino-esp32/issues/6140
    ledcAttachPin(PIN_LEDR, RED_CHANNEL);
    ledcAttachPin(PIN_LEDG, GREEN_CHANNEL);
    ledcAttachPin(PIN_LEDB, BLUE_CHANNEL);
    ledcAttachPin(PIN_LCD_LED, LCD_CHANNEL);

    // channel 0, duty cycle 100%
    ledcWrite(CP_CHANNEL, CONTROL_PILOT_DUTYCICLE_100);
    ledcWrite(RED_CHANNEL, 255);
    ledcWrite(GREEN_CHANNEL, 0);
    ledcWrite(BLUE_CHANNEL, 255);
    ledcWrite(LCD_CHANNEL, 0);

    // Setup PIN interrupt on rising edge
    // the timer interrupt will be reset in the ISR.
    attachInterrupt(PIN_CP_OUT, onCPpulseInterrupt, RISING);

    // Uart 1 is used for Modbus @ 9600 8N1
    Serial1.begin(MODBUS_BAUDRATE, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

    // Check type of calibration value used to characterize ADC
    EVSELogger::debug("[EVSEController] Checking eFuse Vref settings: ");
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        EVSELogger::debug("[EVSEController] Checking eFuse Vref settings: OK");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        EVSELogger::debug("[EVSEController] Checking eFuse Vref settings: Two Point");
    } else {
        EVSELogger::debug("[EVSEController] Checking eFuse Vref settings: not programmed!!!");
    }

    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        EVSELogger::error("[EVSEController] SPIFFS failed! Already tried formatting. HALT");
        while (true) {
            delay(1);
        }
    }

    sprintf(sprintfStr, "[EVSEController] Total SPIFFS bytes: %u, Bytes used: %u\n", SPIFFS.totalBytes(),
            SPIFFS.usedBytes());
    EVSELogger::debug(sprintfStr);

    readEpromSettings();
    onStandbyReadyToCharge();
}

void EVSEController::loop() {
#if EVSE_FEATFLAG_ENABLE_EXTSWITCH
    sampleExternalSwitch();
#endif
#if EVSE_FEATFLAG_ENABLE_RCMON
    sampleRCMSensor();
#endif
    sampleControlPilotLine();

    // once a second, measure temperature (range -40 .. +125C)
    if ((millis() - temperatureLastSampleMillis) >= TEMPERATURE_SAMPLE_PERIOD_MILLIS) {
        sampleTemperatureSensor();
    }

    //  Decrease Charge Delay counter
    if (chargeDelaySeconds > 0) {
        if ((millis() - chargeDelayLastMillis) >= 1000) {
            chargeDelayLastMillis = millis();
            chargeDelaySeconds--;
        }
    }

    //  Decrease Solar stop timer
    if (solarStopTimer > 0) {
        if ((millis() - solarStopTimerLastMillis) >= 1000) {
            solarStopTimerLastMillis = millis();
            if (--solarStopTimer == 0) {
                onSolarStopTimer();
            }
        }
    }

    if (!isEnoughPower()) {
        if (evseCluster.amIMasterOrLBDisabled() && evseCluster.isEnoughCurrentAvailableForAnotherEVSE()) {
            onPowerBackOn();
        } else {
            waitForEnoughPower();
        }
    }

    evseWorkflow.loop();
}

EVSEController evseController;