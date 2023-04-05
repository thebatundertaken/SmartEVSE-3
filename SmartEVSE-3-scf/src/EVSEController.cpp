#include <Arduino.h>
#include <SPI.h>

#include <SPIFFS.h>

#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "driver/uart.h"
#include "soc/rtc_io_struct.h"

#include <Preferences.h>

#include "EVSEController.h"
#include "EVSELogger.h"
#include "EVSEModbus.h"
#include "EVSEPin.h"
#include "EVSERFID.h"

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
const char* PREFS_ICAL_KEY = "ICal";
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

// Alarm interrupt handler
// in STATE A this is called every 1ms (autoreload)
// in STATE B/C there is a PWM signal, and the Alarm is set to 5% after the low-> high transition of the PWM signal
void IRAM_ATTR EVSEController::onTimerA() {
    evseController.sampleADC();
}

// CP pin low to high transition ISR
void IRAM_ATTR EVSEController::onCPpulseInterrupt() {
    evseController.onCPpulse();
}

void EVSEController::onDiodeOK() {
    EVSELogger::debug("Diode OK");
    isDiodeOk = true;
    // Enable Timer alarm, set to start of CP signal (5%)
    timerAlarmWrite(timerA, PWM_5, false);
}

bool EVSEController::isVehicleConnected() {
    // when access bit = 1, p.ex. in OFF mode, the STATEs are no longer updated
    return controlPilot != CONTROL_PILOT_12V;
}

// Set EVSE mode
void EVSEController::setMode(uint8_t newMode) {
    mode = newMode;
    // Disable modbus reception on normal mode
    if (mode == MODE_NORMAL) {
        evseModbus.mainsMeter = MAINS_METER_DISABLED;
        evseModbus.pvMeter = PV_METER_DISABLED;
    }

    evseModbus.broadcastNewModeToNodes(mode);
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

    // adc_ll_amp_disable()  // Close ADC AMP module if don't use it for power save.
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

//  Tell EV to stop charging. When we are not charging switch to State B1
void EVSEController::stopChargingOnError() {
    if (state == STATE_C_CHARGING) {
        setState(STATE_C1_CHARGING_NO_POWER);
    } else if (state == STATE_B_VEHICLE_DETECTED) {
        setState(STATE_B1_VEHICLE_DETECTED_NO_POWER);
    }
}

void EVSEController::onNodeReceivedError(uint8_t newErrorFlags) {
    errorFlags = newErrorFlags;

    if (errorFlags) {
        stopChargingOnError();
        chargeDelaySeconds = DEFAULT_CHARGE_DELAY_SECONDS;
    }
}

void EVSEController::setAccess(bool access) {
    evseRFID.rfidAccessBit = access ? RFID_ACCESS_GRANTED : RFID_NO_ACCESS;

    if (!access) {
        stopChargingOnError();
    }
}

void EVSEController::onCTCommunicationLost() {
    errorFlags |= ERROR_FLAG_CT_NOCOMM;
    stopChargingOnError();
}

void EVSEController::waitForEnoughPower() {
    stopChargingOnError();
    // Set Chargedelay
    chargeDelaySeconds = DEFAULT_CHARGE_DELAY_SECONDS;
}

// Set Charge Current
// Current in Amps * 10 (160 = 16A)
void EVSEController::setCurrent(uint16_t current) {
    uint32_t DutyCycle;

    if ((current >= 60) && (current <= 510)) {
        DutyCycle = current / 0.6;
        // calculate DutyCycle from current
    } else if ((current > 510) && (current <= 800)) {
        DutyCycle = (current / 2.5) + 640;
    } else {
        DutyCycle = 100;  // invalid, use 6A
    }

    // conversion to 1024 = 100%
    DutyCycle = DutyCycle * 1024 / 1000;
    // update PWM signal
    ledcWrite(CP_CHANNEL, DutyCycle);
}

// Change from Solar to Smart mode and vice versa
void EVSEController::switchModeSolarSmart() {
    setMode(~mode & 0x3);
    // Clear All errors
    errorFlags &= ~(ERROR_FLAG_NO_SUN | ERROR_FLAG_LESS_6A);
    // Clear any Chargedelay
    chargeDelaySeconds = 0;
    // Also make sure the SolarTimer is disabled.
    setSolarStopTimer(0);
}

bool EVSEController::isEnoughPower() {
    // Check flags no_sun and less_6a are off
    return !(errorFlags & (ERROR_FLAG_NO_SUN | ERROR_FLAG_LESS_6A));
}

// Not enough power
void EVSEController::onNotEnoughPower() {
    errorFlags |= (mode == MODE_SOLAR) ? ERROR_FLAG_NO_SUN : ERROR_FLAG_LESS_6A;
}

void EVSEController::onPowerBackOn() {
    if (!evseModbus.amIMasterOrDisabled()) {
        return;
    }

    errorFlags &= ~ERROR_FLAG_LESS_6A;
    errorFlags &= ~ERROR_FLAG_NO_SUN;

    evseModbus.broadcastErrorFlagsToNodes(errorFlags);
}

void EVSEController::setState(uint8_t NewState) {
    switch (NewState) {
        case STATE_B1_VEHICLE_DETECTED_NO_POWER:
            // When entering State B1, wait at least 3 seconds before switching to another state.
            if (!chargeDelaySeconds) {
                chargeDelaySeconds = 3;
            }
            break;

        case STATE_A_STANDBY:
            CONTACTOR1_OFF;
            CONTACTOR2_OFF;
            // PWM off,  channel 0, duty cycle 100%
            ledcWrite(CP_CHANNEL, 1024);
            // Alarm every 1ms, auto reload
            timerAlarmWrite(timerA, PWM_100, true);
            if (NewState == STATE_A_STANDBY) {
                errorFlags &= ~ERROR_FLAG_NO_SUN;
                errorFlags &= ~ERROR_FLAG_LESS_6A;
                // Clear charge delay when disconnected
                chargeDelaySeconds = 0;
                evseModbus.resetNode(0);
            }
            break;

        case STATE_B_VEHICLE_DETECTED:
            CONTACTOR1_OFF;
            CONTACTOR2_OFF;
            // Enable Timer alarm, set to diode test (95%)
            timerAlarmWrite(timerA, PWM_95, false);
            // Enable PWM
            setCurrent(chargeCurrent);
            break;

        case STATE_C_CHARGING:
            CONTACTOR1_ON;
            CONTACTOR2_ON;
            break;

        case STATE_C1_CHARGING_NO_POWER:
            // PWM off,  channel 0, duty cycle 100%
            ledcWrite(CP_CHANNEL, 1024);
            // Alarm every 1ms, auto reload. EV should detect and stop charging within 3 seconds
            timerAlarmWrite(timerA, PWM_100, true);
            // Wait maximum 6 seconds (CHARGING_NO_POWER_WAIT_TIMEOUT_MILLIS), before forcing the contactor off
            chargingNoPwmTimeoutMillis = millis();
            chargeDelaySeconds = 15;
            break;
    }

    state = NewState;
    evseModbus.setMasterNodeBalancedState(state);
}

// Sample the Temperature sensor
void EVSEController::sampleTemperatureSensor() {
    temperatureLastSampleMillis = millis();

    RTC_ENTER_CRITICAL();
    uint32_t sample = local_adc1_read(ADC1_CHANNEL_0);
    RTC_EXIT_CRITICAL();

    // voltage range is from 0-2200mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(sample, adc_chars_Temperature);

    // The MCP9700A temperature sensor outputs 500mV at 0C, and has a 10mV/C change in output voltage.
    // so 750mV is 25C, 400mV = -10C
    temperature = (signed int)(voltage - 500) / 10;
    // Serial.printf("\nTemp: %i C (%u mV) ", Temperature , voltage);

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
        evseModbus.resetBalancedStates();
    }
}

// Determine the state of the Pilot signal
void EVSEController::sampleControlPilotLine() {
    uint32_t sample;
    uint32_t min = 3300;
    uint32_t max = 0;
    uint32_t voltage;

    // make sure we wait 100ms after each state change before calculating Average
    // if ( (StateTimer + 100) > millis() ) return CONTROL_PILOT_WAIT;

    // calculate Min/Max of last 25 CP measurements
    for (uint8_t n = 0; n < 25; n++) {
        sample = ADCsamples[n];
        // convert adc reading to voltage
        voltage = esp_adc_cal_raw_to_voltage(sample, adc_chars_CP);
        if (voltage < min) {
            // store lowest value
            min = voltage;
        }

        if (voltage > max) {
            // store highest value
            max = voltage;
        }
    }
    // Serial.printf("min:%u max:%u\n",Min ,Max);

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

void EVSEController::statesWorkflowStandby() {
    // Disconnected or no PWM, or forced to State A, but still connected to the EV
    if (controlPilot == CONTROL_PILOT_12V) {
        // If the RFID reader is set to EnableOne mode, and the Charging cable is disconnected
        // We start a timer to re-lock the EVSE (and unlock the cable) after 60 seconds.
        // evseRFID.startLockWait();

        if (state != STATE_A_STANDBY) {
            // reset state, incase we were stuck in STATE_MODBUS_COMM_B
            setState(STATE_A_STANDBY);
        }

        // Clear chargeDelaySeconds when disconnected.
        chargeDelaySeconds = 0;

        evseModbus.evMeterResetKwhOnStandby();

        return;
    }

    // switch to State B ?
    if (controlPilot == CONTROL_PILOT_9V && errorFlags == ERROR_FLAG_NO_ERROR && chargeDelaySeconds == 0 && evseRFID.isRFIDAccessGranted() &&
        state != STATE_MODBUS_COMM_B) {
        // Allow to switch to state C directly if STATE_A_TO_C is set to CONTROL_PILOT_6V (see EVSE.h)
        isDiodeOk = false;

        // Sample Proximity Pin
        sampleProximityPilot();
        calcChargeCurrent();

        // For LB nodes only: send command to Master, followed by Max Charge Current
        if (evseModbus.amIWorkerNode()) {
            // Node wants to switch to State B
            setState(STATE_MODBUS_COMM_B);

            // Load Balancing: Master or Disabled
            return;
        }

        // Is there any power?
        if (evseModbus.isCurrentAvailable()) {
            evseModbus.setMasterNodeBalancedMax(cableMaxCapacity * 10);
            // Set controlPilot duty cycle to ChargeCurrent (v2.15)
            evseModbus.setMasterNodeBalancedCurrent(chargeCurrent);
            // switch to State B
            setState(STATE_B_VEHICLE_DETECTED);
            // Turn off current if unused for 30 seconds
            currentAvailableWaitMillis = millis();
            return;
        }

        // Not enough power
        onNotEnoughPower();
    }
}

void EVSEController::statesWorkflowDetected() {
    // Disconnected or no PWM
    if (controlPilot == CONTROL_PILOT_12V) {
        setState(STATE_A_STANDBY);
        return;
    }

    if (controlPilot == CONTROL_PILOT_6V) {
        if (isDiodeOk && (errorFlags == ERROR_FLAG_NO_ERROR) && (chargeDelaySeconds == 0)) {
            evseModbus.evMeterResetKwhOnCharging();

            // Load Balancing : Node
            if (evseModbus.amIWorkerNode()) {
                if (state != STATE_MODBUS_COMM_C) {
                    // Send command to Master, followed by Charge Current
                    setState(STATE_MODBUS_COMM_C);
                }

                // Load Balancing: Master or Disabled
                return;
            }

            evseModbus.setMasterNodeBalancedMax(chargeCurrent);
            if (evseModbus.isCurrentAvailable()) {
                // For correct baseload calculation set current to zero
                evseModbus.setMasterNodeBalancedCurrent(0);
                // Calculate charge current for all connected EVSE's
                evseModbus.calcBalancedCurrent(CALC_BALANCED_CURRENT_MODE_ALL_EVSES);
                setState(STATE_C_CHARGING);

                // Vehicle is charging, disable disconnect timeout
                currentAvailableWaitMillis = 0;
                isDiodeOk = false;

                /*if (!evseMenu.currentMenuOption) {
                    // Don't update the LCD if we are navigating the menu
                    // GLCD();
                }*/

                return;
            }

            // Not enough power
            onNotEnoughPower();
        }

        return;
    }

    // Current NOT in use after 30 seconds, change state to ACTIVATION_IN_PROGRESS and wait 3 more seconds
    if (currentAvailableWaitMillis != 0 && (millis() - currentAvailableWaitMillis) >= CURRENTAVAILABLE_WAIT_TIMEOUT_MILLIS) {
        currentAvailableWaitMillis = 0;
        setState(STATE_DISCONNECT_IN_PROGRESS);
        // Timer 3 seconds, in order to let car the process electric signal on the fly, before disconnecting
        disconnectRequestMillis = millis();
        // PWM off, channel 0, duty cycle 0%
        // Control controlPilot static -12V
        ledcWrite(CP_CHANNEL, 0);
    }

    if (controlPilot == CONTROL_PILOT_DIODE_CHECK_OK) {
        // Diode found, OK
        onDiodeOK();
    }
}

void EVSEController::statesWorkflow() {
    switch (state) {
        case STATE_A_STANDBY:
        case STATE_B1_VEHICLE_DETECTED_NO_POWER:
        case STATE_MODBUS_COMM_B:
            statesWorkflowStandby();
            break;

        case STATE_MODBUS_COMM_B_OK:
            setState(STATE_B_VEHICLE_DETECTED);
            // Turn off current if unused for 30 seconds
            currentAvailableWaitMillis = millis();
            break;

        case STATE_B_VEHICLE_DETECTED:
        case STATE_MODBUS_COMM_C:
            statesWorkflowDetected();
            break;

        case STATE_C1_CHARGING_NO_POWER:
            // Disconnected or no PWM
            if (controlPilot == CONTROL_PILOT_12V) {
                chargingNoPwmTimeoutMillis = 0;
                setState(STATE_A_STANDBY);
            } else if (controlPilot == CONTROL_PILOT_9V) {
                chargingNoPwmTimeoutMillis = 0;
                setState(STATE_B1_VEHICLE_DETECTED_NO_POWER);
            } else {
                // if the EV does not stop charging in 6 seconds, we will open the contactor.
                if ((chargingNoPwmTimeoutMillis > 0) && ((millis() - chargingNoPwmTimeoutMillis) >= CHARGING_NO_POWER_WAIT_TIMEOUT_MILLIS)) {
                    chargingNoPwmTimeoutMillis = 0;
                    EVSELogger::info("State C1 Timeout!");
                    setState(STATE_B1_VEHICLE_DETECTED_NO_POWER);
                }
            }
            break;

        case STATE_MODBUS_COMM_C_OK:
            setState(STATE_C_CHARGING);
            // Vehicle is charging, disable disconnect timeout
            currentAvailableWaitMillis = 0;
            isDiodeOk = 0;
            break;

        case STATE_C_CHARGING:
            // Disconnected or no PWM
            if (controlPilot == CONTROL_PILOT_12V) {
                setState(STATE_A_STANDBY);
            } else if (controlPilot == CONTROL_PILOT_9V) {
                setState(STATE_B_VEHICLE_DETECTED);
                isDiodeOk = 0;
            }
            break;

        case STATE_DISCONNECT_IN_PROGRESS:
            // Timer 3 seconds, in order to let car the process electric signal on the fly, before disconnecting
            if (disconnectRequestMillis != 0 && (millis() - disconnectRequestMillis) >= DISCONNECT_WAIT_MILLIS) {
                disconnectRequestMillis = 0;
                setState(STATE_B_VEHICLE_DETECTED);
            }
            break;
    }
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

        RCmon = preferences.getUChar(PREFS_RCMON_KEY, RC_MON_DISABLED);
        ICal = preferences.getUShort(PREFS_ICAL_KEY, ICAL_DEFAULT);
        mode = preferences.getUChar(PREFS_MODE_KEY, MODE_NORMAL);
        config = preferences.getUChar(PREFS_CONFIG_KEY, CONFIG_SOCKET);
        externalSwitch = preferences.getUChar(PREFS_SWITCH_KEY, SWITCH_DISABLED);
        maxCurrent = preferences.getUShort(PREFS_MAXCURRENT_KEY, MAX_CURRENT);
        minCurrent = preferences.getUShort(PREFS_MINCURRENT_KEY, MIN_CURRENT);
        maxMains = preferences.getUShort(PREFS_MAXMAINS_KEY, MAX_MAINS);
        solarStartCurrent = preferences.getUShort(PREFS_STARTCURRENT_KEY, SOLAR_START_CURRENT);
        solarStopTime = preferences.getUShort(PREFS_STOPTIME_KEY, SOLAR_STOP_TIME_MINUTES);
        solarImportCurrent = preferences.getUShort(PREFS_IMPORTCURRENT_KEY, SOLAR_IMPORT_CURRENT);
        maxTemperature = preferences.getChar(PREFS_MAXTEMP_KEY, DEFAULT_MAX_TEMPERATURE);
    }
    preferences.end();

    if (firstRun) {
        RCmon = RC_MON_DISABLED;
        ICal = ICAL_DEFAULT;
        mode = MODE_NORMAL;
        config = CONFIG_SOCKET;
        externalSwitch = SWITCH_DISABLED;
        maxCurrent = MAX_CURRENT;
        minCurrent = MIN_CURRENT;
        maxMains = MAX_MAINS;
        solarStartCurrent = SOLAR_START_CURRENT;
        solarStopTime = SOLAR_STOP_TIME_MINUTES;
        solarImportCurrent = SOLAR_IMPORT_CURRENT;
        maxTemperature = DEFAULT_MAX_TEMPERATURE;
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
        // Serial.printf("hwversion %04x serialnr:%u \n",hwversion, serialnr);
        // Serial.print(ec_public);
    } else {
        Serial.print("No KeyStorage found in nvs!\n");
    }*/
}

void EVSEController::writeEpromSettings() {
    Preferences preferences;
    if (preferences.begin(PREFS_CONTROLLER_NAMESPACE, false) != true) {
        EVSELogger::error("Unable to write preferences for EVSEController");
        return;
    }

    preferences.putUChar(PREFS_RCMON_KEY, RCmon);
    preferences.putUShort(PREFS_ICAL_KEY, ICal);
    preferences.putUChar(PREFS_MODE_KEY, mode);
    preferences.putUChar(PREFS_CONFIG_KEY, config);
    preferences.putUChar(PREFS_SWITCH_KEY, externalSwitch);
    preferences.putUShort(PREFS_MAXCURRENT_KEY, maxCurrent);
    preferences.putUShort(PREFS_MINCURRENT_KEY, minCurrent);
    preferences.putUShort(PREFS_MAXMAINS_KEY, maxMains);
    preferences.putUShort(PREFS_STARTCURRENT_KEY, solarStartCurrent);
    preferences.putUShort(PREFS_STOPTIME_KEY, solarStopTime);
    preferences.putUShort(PREFS_IMPORTCURRENT_KEY, solarImportCurrent);
    preferences.putChar(PREFS_MAXTEMP_KEY, maxTemperature);

    preferences.end();
}

void EVSEController::calculateCalibration(uint16_t CT1) {
    // Calculate new Calibration value and set the Irms value
    ICal = ((unsigned long)CT1 * 10 + 5) * ICAL_DEFAULT / Iuncal;
    Irms[0] = CT1;
}

void EVSEController::setSolarStopTimer(uint16_t Timer) {
    if (solarStopTimer == Timer) {
        return;
    }

    solarStopTimer = Timer;
    evseModbus.broadcastSolarStopTimerToNodes(solarStopTimer);
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

// RCM = Residual Current Monitor
// Clear RCM error bit, by pressing any button
void EVSEController::resetRCMErrorFlag() {
    // RCM was tripped, but RCM level is back to normal. Clear RCM error bit, by pressing any button
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

void EVSEController::onExternalSwitchInputPulledLow() {
    switch (externalSwitch) {
        case SWITCH_DISABLED:
            if (state == STATE_C_CHARGING) {
                setState(STATE_C1_CHARGING_NO_POWER);
                // Keep in State B for 15 seconds, so the Charge cable can be removed.
                chargeDelaySeconds = 15;
            }
            break;

        case SWITCH_ACCESS_BUTTON:
            // Toggle Access bit on/off
            setAccess(!evseRFID.rfidAccessBit);
            break;

        case SWITCH_ACCESS_SWITCH:
            setAccess(true);
            break;

        case SWITCH_SMARTSOLAR_BUTTON:
            // Smart-Solar Button or hold button for 1,5 second to STOP charging
            if (RB2Timer == 0) {
                RB2Timer = millis();
            }

            if (RB2Timer != 0 && ((millis() - RB2Timer) > SWITCH_SMARTSOLAR_BUTTON_LONGPRESSED_MILLIS)) {
                if (state == STATE_C_CHARGING) {
                    setState(STATE_C1_CHARGING_NO_POWER);
                    // Keep in State B for 15 seconds, so the Charge cable can be removed.
                    chargeDelaySeconds = 15;
                }
            }
            break;

        case SWITCH_SMARTSOLAR_SWITCH:
            if (mode == MODE_SOLAR) {
                switchModeSolarSmart();
            }
            break;
    }

    // Reset RCM error when button is pressed. RCM was tripped, but RCM level is back to normal
    resetRCMErrorFlag();
}

void EVSEController::onExternalSwitchInputReleased() {
    switch (externalSwitch) {
        case SWITCH_ACCESS_SWITCH:
            setAccess(false);
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
        onExternalSwitchInputPulledLow();
    } else {
        onExternalSwitchInputReleased();
    }
}

void EVSEController::calcChargeCurrent() {
    // Do not exceed cable max capacity
    chargeCurrent = min(maxCurrent, cableMaxCapacity) * 10;
}

// Sample the Proximity Pin, and determine the maximum current the cable can handle.
void EVSEController::sampleProximityPilot() {
    // PP will be sample only for CONFIG_SOCKET mode, since PP cable is disconnect for CONFIG_FIXED_CABLE
    if (config == CONFIG_FIXED_CABLE) {
        cableMaxCapacity = maxCurrent;
        return;
    }

    RTC_ENTER_CRITICAL();
    uint32_t sample = local_adc1_read(ADC1_CHANNEL_6);
    RTC_EXIT_CRITICAL();

    uint32_t voltage = esp_adc_cal_raw_to_voltage(sample, adc_chars_PP);
    // Serial.printf("PP pin: %u (%u mV)\n", sample, voltage);

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

    // No resistor, Max cable current = 13A
    cableMaxCapacity = MAX_CURRENT;
}

void EVSEController::setup() {
    pinMode(PIN_CP_OUT, OUTPUT);  // CP output
    pinMode(PIN_SW_IN, INPUT);    // SW Switch input
    pinMode(PIN_SSR, OUTPUT);     // SSR1 output
    pinMode(PIN_SSR2, OUTPUT);    // SSR2 output
    pinMode(PIN_RCM_FAULT, INPUT_PULLUP);

    pinMode(PIN_LCD_LED, OUTPUT);  // LCD backlight
    pinMode(PIN_LCD_RST, OUTPUT);  // LCD reset

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
    Serial.print("\nSmartEVSE v3 powerup\n");

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
    adc1_config_width(ADC_WIDTH_BIT_10);                         // 10 bits ADC resolution is enough
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);  // setup the CP pin input attenuation to 11db
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_6);   // setup the PP pin input attenuation to 6db
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_6);   // setup the Temperature input attenuation to 6db

    // Characterize the ADC at particular attentuation for each channel
    adc_chars_CP = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    adc_chars_PP = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    adc_chars_Temperature = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_10, 1100, adc_chars_CP);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_10, 1100, adc_chars_PP);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_10, 1100, adc_chars_Temperature);

    // Setup PWM on channel 0, 1000Hz, 10 bits resolution
    ledcSetup(CP_CHANNEL, 1000, 10);  // channel 0  => Group: 0, Channel: 0, Timer: 0
    // setup the RGB led PWM channels
    // as PWM channel 1 is used by the same timer as the CP timer (channel 0), we start with channel 2
    ledcSetup(RED_CHANNEL, 5000, 8);    // R channel 2, 5kHz, 8 bit
    ledcSetup(GREEN_CHANNEL, 5000, 8);  // G channel 3, 5kHz, 8 bit
    ledcSetup(BLUE_CHANNEL, 5000, 8);   // B channel 4, 5kHz, 8 bit

    // attach the channels to the GPIO to be controlled
    ledcAttachPin(PIN_CP_OUT, CP_CHANNEL);
    // pinMode(PIN_CP_OUT, OUTPUT);                // Re-init the pin to output, required in order for attachInterrupt to work (2.0.2)
    //  not required/working on master branch..
    //  see https://github.com/espressif/arduino-esp32/issues/6140
    ledcAttachPin(PIN_LEDR, RED_CHANNEL);
    ledcAttachPin(PIN_LEDG, GREEN_CHANNEL);
    ledcAttachPin(PIN_LEDB, BLUE_CHANNEL);

    // channel 0, duty cycle 100%
    ledcWrite(CP_CHANNEL, 1024);
    ledcWrite(RED_CHANNEL, 255);
    ledcWrite(GREEN_CHANNEL, 0);
    ledcWrite(BLUE_CHANNEL, 255);

    // Setup PIN interrupt on rising edge
    // the timer interrupt will be reset in the ISR.
    attachInterrupt(PIN_CP_OUT, onCPpulseInterrupt, RISING);

    // Uart 1 is used for Modbus @ 9600 8N1
    Serial1.begin(MODBUS_BAUDRATE, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

    // Check type of calibration value used to characterize ADC
    Serial.print("Checking eFuse Vref settings: ");
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.print("OK\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.print("Two Point\n");
    } else {
        Serial.print("not programmed!!!\n");
    }

    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.print("SPIFFS failed! Already tried formatting. HALT\n");
        while (true) {
            delay(1);
        }
    }
    Serial.printf("Total SPIFFS bytes: %u, Bytes used: %u\n", SPIFFS.totalBytes(), SPIFFS.usedBytes());

    readEpromSettings();
}

void EVSEController::loop() {
    sampleExternalSwitch();
    sampleRCMSensor();
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
                errorFlags |= ERROR_FLAG_NO_SUN;
                stopChargingOnError();

                // reset all states
                evseModbus.resetBalancedStates();
            }
        }
    }

    if (!isEnoughPower()) {
        if (evseModbus.amIMasterOrDisabled() && evseModbus.isCurrentAvailable()) {
            onPowerBackOn();
        } else {
            waitForEnoughPower();
        }
    }

    // EVSEStates10ms
    statesWorkflow();
}

EVSEController evseController;