#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include "driver/uart.h"

#include <Preferences.h>

#include "EVSELogger.h"
#include "EVSEMenu.h"
#include "EVSEModbus.h"
#include "utils.h"

#define MODBUS_REG_ERRORFLAGS 0x0001
#define MODBUS_REG_NEWMODE 0x0003
#define MODBUS_REG_SOLARSTOPTIMER 0x0004
#define MODBUS_REG_RESET 0x0006
#define MODBUS_REG_BALANCECURRENTS 0x0020

#define MODBUS_START_MODE 255

#define TRANSFORMER_COMP 100

const char* PREFS_CLUSTER_NAMESPACE = "settings";
const char* PREFS_LOADBL_KEY = "LoadBl";
const char* PREFS_EVMETER_KEY = "EVMeter";
const char* PREFS_EVMETERADDRESS_KEY = "EVMeterAddress";
const char* PREFS_PVMETER_KEY = "PVMeter";
const char* PREFS_PVMADDRESS_KEY = "PVMAddress";
const char* PREFS_GRID_KEY = "Grid";
const char* PREFS_MAINSMETERADDRESS_KEY = "MainsMAddress";
const char* PREFS_MAINSMETERMEASURE_KEY = "MainsMMeasure";
const char* PREFS_MAINSMETER_KEY = "MainsMeter";
const char* PREFS_MAXCIRCUIT_KEY = "MaxCircuit";

bool EVSEModbus::isGridActive() {
    return gridActive;
}

void EVSEModbus::setGridActive(bool val) {
    gridActive = val;
}

bool EVSEModbus::isCalibrationActive() {
    return calibrationActive;
}

void EVSEModbus::setMasterNodeBalancedState(uint8_t state) {
    balancedState[0] = state;
}

void EVSEModbus::setMasterNodeBalancedMax(uint16_t newBalancedMax) {
    balancedMax[0] = newBalancedMax;
}

void EVSEModbus::setMasterNodeBalancedCurrent(uint16_t current) {
    balancedCurrent[0] = current;
}

void EVSEModbus::evMeterResetKwhOnCharging() {
    if (evMeter != EV_METER_DISABLED && evMeterResetKwh != EVMETER_RESET_KWH_OFF) {
        // store kwh measurement at start of charging.
        evMeterEnergyAtStart = evMeterEnergy;
        // clear flag, will be set when disconnected from EVSE (State A)
        evMeterResetKwh = EVMETER_RESET_KWH_OFF;
    }
}

void EVSEModbus::evMeterResetKwhOnStandby() {
    // Do not reset on powerup (evMeterResetKwh == EVMETER_RESET_KWH_POWERUP)
    if (evMeterResetKwh == EVMETER_RESET_KWH_OFF) {
        // when set, reset EV kWh meter on state B->C change.
        evMeterResetKwh = EVMETER_RESET_KWH_ON;
    }
}

void EVSEModbus::setPvMeter(uint8_t value) {
    pvMeter = (mainsMeterMeasure == MAINS_METER_MEASURE) ? PV_METER_DISABLED : value;
}

void EVSEModbus::modbusOnClientError(Error error, uint32_t token) {
    // ModbusError wraps the error code and provides a readable error message for it
    ModbusError me(error);
    // Serial.printf("Error response: %02X - %s\n", error, (const char *)me);
}

bool EVSEModbus::amIMasterOrDisabled() {
    return LoadBl == LOAD_BALANCER_MASTER || LoadBl == LOAD_BALANCER_DISABLED;
}

// Cluster mode: node 1 = master, workers 2-8
bool EVSEModbus::amIWorkerNode() {
    return LoadBl >= 2;
}

bool EVSEModbus::isLoadBalancerMaster() {
    return LoadBl == LOAD_BALANCER_MASTER;
}

bool EVSEModbus::isLoadBalancerDisabled() {
    return LoadBl == LOAD_BALANCER_DISABLED;
}

void EVSEModbus::broadcastSysConfigToNodes(uint16_t values[]) {
    if (isLoadBalancerMaster()) {
        ModbusWriteMultipleRequest(BROADCAST_ADR, MODBUS_SYS_CONFIG_START, values, MODBUS_SYS_CONFIG_COUNT);
    }
}

void EVSEModbus::broadcastErrorFlagsToNodes(uint16_t flags) {
    if (isLoadBalancerMaster()) {
        ModbusWriteSingleRequest(BROADCAST_ADR, MODBUS_REG_ERRORFLAGS, flags);
    }
}

void EVSEModbus::broadcastSolarStopTimerToNodes(uint16_t value) {
    if (isLoadBalancerMaster()) {
        ModbusWriteSingleRequest(BROADCAST_ADR, MODBUS_REG_SOLARSTOPTIMER, value);
    }
}

void EVSEModbus::broadcastNewModeToNodes(uint16_t newMode) {
    if (isLoadBalancerMaster()) {
        ModbusWriteSingleRequest(BROADCAST_ADR, MODBUS_REG_NEWMODE, newMode);
    }
}

void EVSEModbus::broadcastResetToNode(uint8_t nodeNr) {
    if (isLoadBalancerMaster() && nodeNr >= 1 && nodeNr <= 7) {
        ModbusWriteSingleRequest(nodeNr + 1u, MODBUS_REG_RESET, 0);
    }
}

// Broadcast momentary currents to all Node EVSE's
void EVSEModbus::broadcastMasterNodeCurrent() {
    ModbusWriteMultipleRequest(BROADCAST_ADR, MODBUS_REG_BALANCECURRENTS, balancedCurrent, NR_EVSES);
}

void EVSEModbus::setOverrideCurrent(uint16_t val) {
    overrideCurrent = val;
    // reset CT timeout when register is written
    lastCTResponseMillis = millis();
}

void EVSEModbus::resetBalancedStates() {
    for (uint8_t n = 1; n < NR_EVSES; n++) {
        // Yes, disable old active Node states
        balancedState[n] = STATE_A_STANDBY;
        // reset ChargeCurrent to 0
        balancedCurrent[n] = 0;
    }
}

void EVSEModbus::onCTCommunicationLost() {
    // Do not re-run if comms are already down
    if (evseController.errorFlags & ERROR_FLAG_CT_NOCOMM) {
        return;
    }

    evseController.onCTCommunicationLost();

    broadcastErrorFlagsToNodes(evseController.errorFlags);
    resetBalancedStates();
}

void EVSEModbus::resetCTDataMeasured() {
    if (evseController.mode != MODE_NORMAL) {
        return;
    }

    // No measurements, so we set it to zero
    imeasured = 0;
    // reset CT counter (not checked for Master)
    lastCTResponseMillis = millis();
}

void EVSEModbus::onCTDataReceived() {
    // Smart/Solar mode
    if (evseController.mode != MODE_NORMAL) {
        // Communication with Sensorbox /Kwh meter OK. Update CT data and broadcast to all connected EVSE's
        if (!(evseController.errorFlags & ERROR_FLAG_CT_NOCOMM)) {
            updateCurrentData();
        }
    } else {
        // Normal Mode
        // Calculate charge current for connected EVSE's
        calcBalancedCurrent(CALC_BALANCED_CURRENT_MODE_ALL_EVSES);
        // Send to all EVSE's (only in Master mode)
        if (isLoadBalancerMaster()) {
            broadcastMasterNodeCurrent();
        }
    }

    // Adjust EVSE current, using Master PWM
    if (evseController.state == STATE_B_VEHICLE_DETECTED || evseController.state == STATE_C_CHARGING) {
        evseController.setCurrent(balancedCurrent[0]);
    }
}

// Update current data after received current measurement
void EVSEModbus::updateCurrentData() {
    // imeasured holds highest Irms of all channels
    imeasured = 0;
    for (uint8_t x = 0; x < 3; x++) {
        if (evseController.Irms[x] > imeasured)
            imeasured = evseController.Irms[x];
    }

    if (amIWorkerNode()) {
        return;
    }

    // Calculate dynamic charge current for connected EVSE's
    calcBalancedCurrent(CALC_BALANCED_CURRENT_MODE_ALL_EVSES);

    // No current left, or Overload (2x Maxmains)?
    if (noCurrent > 2 || (imeasured > (evseController.maxMains * 20))) {
        // STOP charging for all EVSE's and display error message: NOCURRENT
        evseController.errorFlags |= ERROR_FLAG_LESS_6A;
        // Set all EVSE's to State A
        resetBalancedStates();

        // Broadcast Error code over RS485
        ModbusWriteSingleRequest(BROADCAST_ADR, MODBUS_FUNCTION_WRITE_ERROR_CODE_REGISTER, ERROR_FLAG_LESS_6A);
        noCurrent = 0;
    } else if (isLoadBalancerMaster()) {
        // Master sends current to all connected EVSE's
        broadcastMasterNodeCurrent();
    }
}

// Calculates balancedCurrent PWM current for each EVSE
// mod =0 normal
// mod =1 we have a new EVSE requesting to start charging.
void EVSEModbus::calcBalancedCurrent(uint8_t mode) {
    int Average, MaxBalanced, Idifference;
    int BalancedLeft = 0;
    signed int IsumImport;
    int ActiveMax = 0, TotalCurrent = 0, Baseload;
    char CurrentSet[NR_EVSES] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t n;

    // Load balancing disabled?, Reset States
    if (!isLoadBalancerDisabled()) {
        resetBalancedStates();
    }

    // Override current temporary if set (from Modbus)
    if (overrideCurrent) {
        evseController.chargeCurrent = overrideCurrent;
    } else {
        evseController.calcChargeCurrent();
    }

    // Load Balancing Disabled or Master:
    if (amIMasterOrDisabled()) {
        // update balancedMax[0] if the MAX current was adjusted using buttons or CLI
        balancedMax[0] = evseController.chargeCurrent;
    }

    for (n = 0; n < NR_EVSES; n++)
        if (balancedState[n] == STATE_C_CHARGING) {
            // Count nr of Active (Charging) EVSE's
            BalancedLeft++;
            // Calculate total Max Amps for all active EVSEs
            ActiveMax += balancedMax[n];
            // Calculate total of all set charge currents
            TotalCurrent += balancedCurrent[n];
        }

    // Normal and Smart mode
    if (mode == CALC_BALANCED_CURRENT_MODE_NORMAL && evseController.mode != MODE_SOLAR) {
        Idifference = (evseController.maxMains * 10) - imeasured;  // Difference between MaxMains and Measured current (can be negative)

        if (Idifference > 0) {
            // increase with 1/4th of difference (slowly increase current)
            isetBalanced += (Idifference / 4);
        } else {
            isetBalanced += (Idifference * 100 / TRANSFORMER_COMP);
            // last PWM setting + difference (immediately decrease current)
        }
        if (isetBalanced < 0) {
            isetBalanced = 0;
        }
        if (isetBalanced > 800) {
            // hard limit 80A (added 11-11-2017)
            isetBalanced = 800;
        }
    }

    if (evseController.mode == MODE_SOLAR) {
        // Allow Import of power from the grid when solar charging
        IsumImport = Isum - (10 * evseController.solarImportCurrent);

        if (IsumImport < 0) {
            // negative, we have surplus (solar) power available
            // more then 1A available, increase balancedCurrent charge current with 0.5A
            if (IsumImport < -10) {
                isetBalanced = isetBalanced + 5;
            } else {
                // less then 1A available, increase with 0.1A
                isetBalanced = isetBalanced + 1;
            }
        } else {
            // positive, we use more power then is generated
            if (IsumImport > 20) {
                // we use atleast 2A more then available, decrease balancedCurrent charge current.
                isetBalanced = isetBalanced - (IsumImport / 2);
            } else if (IsumImport > 10) {
                // we use 1A more then available, decrease with 0.5A
                isetBalanced = isetBalanced - 5;
            } else if (IsumImport > 3) {
                // we still use > 0.3A more then available, decrease with 0.1A
                // if we use <= 0.3A we do nothing
                isetBalanced = isetBalanced - 1;
            }
        }

        // If isetBalanced is below MinCurrent or negative, make sure it's set to MinCurrent.
        if ((isetBalanced < (BalancedLeft * evseController.minCurrent * 10)) || (isetBalanced < 0)) {
            isetBalanced = BalancedLeft * evseController.minCurrent * 10;
            // ----------- Check to see if we have to continue charging on solar power alone ----------
            if (BalancedLeft && evseController.solarStopTime && (IsumImport > 10)) {
                if (evseController.solarStopTimer == 0) {
                    // Convert minutes into seconds
                    evseController.setSolarStopTimer(evseController.solarStopTime * 60);
                }
            } else {
                evseController.setSolarStopTimer(0);
            }
        } else {
            evseController.setSolarStopTimer(0);
        }
    }
    // When Load balancing = Master,  Limit total current of all EVSEs to MaxCircuit
    if (isLoadBalancerMaster() && (isetBalanced > (maxCircuit * 10))) {
        isetBalanced = maxCircuit * 10;
    }

    Baseload = imeasured - TotalCurrent;  // Calculate Baseload (load without any active EVSE)
    if (Baseload < 0) {
        Baseload = 0;
    }

    if (evseController.mode == MODE_NORMAL) {
        if (isLoadBalancerMaster()) {
            // Load Balancing = Master? MaxCircuit is max current for all active EVSE's
            isetBalanced = maxCircuit * 10;
        } else {
            // No Load Balancing in Normal Mode. Set current to ChargeCurrent (fix: v2.05)
            isetBalanced = evseController.chargeCurrent;
        }
    }

    // Only if we have active EVSE's
    if (BalancedLeft) {
        // New EVSE charging, and no Solar mode
        if (mode == CALC_BALANCED_CURRENT_MODE_ALL_EVSES && evseController.mode != MODE_SOLAR) {
            // Set max combined charge current to MaxMains - Baseload
            isetBalanced = (evseController.maxMains * 10) - Baseload;
        }

        if (isetBalanced < 0 || isetBalanced < (BalancedLeft * evseController.minCurrent * 10) ||
            (evseController.mode == MODE_SOLAR && Isum > 10 && imeasured > (evseController.maxMains * 10))) {
            // set minimal "MinCurrent" charge per active EVSE
            isetBalanced = BalancedLeft * evseController.minCurrent * 10;
            // Flag noCurrent left
            noCurrent++;
        } else {
            noCurrent = 0;
        }

        // limit to total maximum Amps (of all active EVSE's)
        if (isetBalanced > ActiveMax) {
            isetBalanced = ActiveMax;
        }

        // convert to Amps
        MaxBalanced = isetBalanced;

        // Calculate average current per EVSE
        n = 0;
        do {
            // Average current for all active EVSE's
            Average = MaxBalanced / BalancedLeft;

            // Check for EVSE's that have a lower MAX current
            // Active EVSE, and current not yet calculated?
            if ((balancedState[n] == STATE_C_CHARGING) && (!CurrentSet[n]) && (Average >= balancedMax[n])) {
                // Set current to Maximum allowed for this EVSE
                balancedCurrent[n] = balancedMax[n];
                // mark this EVSE as set.
                CurrentSet[n] = 1;
                // decrease counter of active EVSE's
                BalancedLeft--;
                // Update total current to new (lower) value
                MaxBalanced -= balancedCurrent[n];
                // check all EVSE's again
                n = 0;
            } else {
                n++;
            }
        } while (n < NR_EVSES && BalancedLeft);

        // All EVSE's which had a Max current lower then the average are set.
        // Now calculate the current for the EVSE's which had a higher Max current
        n = 0;
        // Any Active EVSE's left?
        if (BalancedLeft) {
            do {
                // Check for EVSE's that are not set yet
                // Active EVSE, and current not yet calculated?
                if ((balancedState[n] == STATE_C_CHARGING) && (!CurrentSet[n])) {
                    // Set current to Average
                    balancedCurrent[n] = MaxBalanced / BalancedLeft;
                    // mark this EVSE as set.
                    CurrentSet[n] = 1;
                    // decrease counter of active EVSE's
                    BalancedLeft--;
                    // Update total current to new (lower) value
                    MaxBalanced -= balancedCurrent[n];
                }
            } while (++n < NR_EVSES && BalancedLeft);
        }

    }  // BalancedLeft
}

// Is there at least 6A(configurable MinCurrent) available for a EVSE?
bool EVSEModbus::isCurrentAvailable() {
    uint8_t activeEVSE = 0;
    int totalCurrent = 0;

    for (uint8_t n = 0; n < NR_EVSES; n++) {
        // must be in STATE_C_CHARGING
        if (balancedState[n] == STATE_C_CHARGING) {
            // Count nr of active (charging) EVSE's
            activeEVSE++;
            // Calculate total max charge current for all active EVSE's
            totalCurrent += balancedCurrent[n];
        }
    }

    // No active (charging) EVSE's
    if (activeEVSE == 0) {
        // There should be at least 6A available
        if (imeasured > ((evseController.maxMains - evseController.minCurrent) * 10)) {
            return false;
        }
    } else {
        // at least one active EVSE
        // Calculate Baseload (load without any active EVSE)
        int baseload = imeasured - totalCurrent;
        if (baseload < 0) {
            // only relevant for Smart/Solar mode
            baseload = 0;
        }

        // Do calculations with one more EVSE
        if (++activeEVSE > NR_EVSES) {
            activeEVSE = NR_EVSES;
        }

        // When load balancing is active, and we are the Master, the Circuit option limits the max total current
        if (isLoadBalancerMaster()) {
            if ((activeEVSE * (evseController.minCurrent * 10)) > (maxCircuit * 10)) {
                return false;
            }
        }

        // Check if the lowest charge current(6A) x ActiveEV's + baseload would be higher then the MaxMains.
        if ((activeEVSE * (evseController.minCurrent * 10) + baseload) > (evseController.maxMains * 10)) {
            return false;
        }
    }

    // Allow solar Charging if surplus current is above 'StartCurrent' (sum of all phases)
    // Charging will start after the timeout (chargedelay) period has ended
    // Only when StartCurrent configured or Node MinCurrent detected or Node inactive
    if (evseController.mode == MODE_SOLAR) {
        // no active EVSE yet?
        if (activeEVSE == 0 && Isum >= ((signed int)evseController.solarStartCurrent * -10)) {
            return false;
        } else if ((activeEVSE * evseController.minCurrent * 10) > totalCurrent) {
            // check if we can split the available current between all active EVSE's
            return false;
        }
    }

    return true;
}

void EVSEModbus::resetNode(uint8_t NodeNr) {
    Node[NodeNr].Timer = 0;
    Node[NodeNr].Phases = 0;
    Node[NodeNr].MinCurrent = 0;
}

/**
 * Master checks node status requests, and responds with new state
 * Master -> Node
 *
 * @param uint8_t NodeAdr (1-7)
 */
void EVSEModbus::processAllNodeStates(uint8_t NodeNr) {
    uint16_t values[2];
    bool write = false;

    values[0] = balancedState[NodeNr];

    bool current = isCurrentAvailable();
    // Yes enough current
    if (current) {
        if (balancedError[NodeNr] & (ERROR_FLAG_LESS_6A | ERROR_FLAG_NO_SUN)) {
            // Clear Error flags
            balancedError[NodeNr] &= ~(ERROR_FLAG_LESS_6A | ERROR_FLAG_NO_SUN);
            write = true;
        }
    }

    // Check EVSE for request to charge states
    switch (balancedState[NodeNr]) {
        case STATE_A_STANDBY:
            resetNode(NodeNr);
            break;

            // Request to charge A->B
        case STATE_MODBUS_COMM_B:
            // check if we have enough current
            if (current) {
                // Mark Node EVSE as active (State B)
                balancedState[NodeNr] = STATE_B_VEHICLE_DETECTED;
                // Initially set current to lowest setting
                balancedCurrent[NodeNr] = evseController.minCurrent * 10;
                values[0] = STATE_MODBUS_COMM_B_OK;
                write = true;
            } else {
                // We do not have enough current to start charging
                // Make sure the Node does not start charging by setting current to 0
                balancedCurrent[NodeNr] = 0;
                // Error flags cleared?
                if ((balancedError[NodeNr] & (ERROR_FLAG_LESS_6A | ERROR_FLAG_NO_SUN)) == 0) {
                    if (evseController.mode == MODE_SOLAR) {
                        // Solar mode: No Solar Power available
                        balancedError[NodeNr] |= ERROR_FLAG_NO_SUN;
                    } else {
                        // Normal or Smart Mode: Not enough current available
                        balancedError[NodeNr] |= ERROR_FLAG_LESS_6A;
                    }
                    write = true;
                }
            }
            break;

            // request to charge B->C
        case STATE_MODBUS_COMM_C:
            balancedCurrent[NodeNr] = 0;
            // For correct baseload calculation set current to zero check if we have enough current
            if (current) {
                // Mark Node EVSE as Charging (State C)
                balancedState[NodeNr] = STATE_C_CHARGING;
                // Calculate charge current for all connected EVSE's
                calcBalancedCurrent(CALC_BALANCED_CURRENT_MODE_ALL_EVSES);
                values[0] = STATE_MODBUS_COMM_C_OK;
                write = true;
            } else {
                // We do not have enough current to start charging
                // Error flags cleared?
                if ((balancedError[NodeNr] & (ERROR_FLAG_LESS_6A | ERROR_FLAG_NO_SUN)) == 0) {
                    if (evseController.mode == MODE_SOLAR) {
                        // Solar mode: No Solar Power available
                        balancedError[NodeNr] |= ERROR_FLAG_NO_SUN;
                    } else {
                        // Normal or Smart Mode: Not enough current available
                        balancedError[NodeNr] |= ERROR_FLAG_LESS_6A;
                    }
                    write = true;
                }
            }
            break;
    }

    values[1] = balancedError[NodeNr];

    if (write) {
        // Write State and Error to Node
        ModbusWriteMultipleRequest(NodeNr + 1, MODBUS_FUNCTION_NODE_STATUS_REGISTER, values, 2);
    }
}

/**
 * Master receives Node configuration over modbus
 * Node -> Master
 *
 * @param uint8_t NodeNr (1-7)
 */
void EVSEModbus::receiveNodeConfig(uint8_t* buf, uint8_t NodeNr) {
    Node[NodeNr].EVMeter = buf[1];
    Node[NodeNr].EVAddress = buf[3];

    // Reset flag on master
    Node[NodeNr].ConfigChanged = 0;
    // Reset flag on node
    ModbusWriteSingleRequest(NodeNr + 1u, MODBUS_REG_RESET, 0);
}

// Monitor EV Meter responses, and update Enery and Power measurements
// Does not send any data back.
ModbusMessage EVSEModbus::onModbusEVMeterResponse(ModbusMessage msg) {
    evseModbus.modbusEVMeterResponseHandler(msg);

    // As this is a response to an earlier request, do not send response.
    return NIL_RESPONSE;
}

void EVSEModbus::modbusEVMeterResponseHandler(ModbusMessage request) {
    ModbusDecode((uint8_t*)request.data(), request.size());

    if (MB.Type != MODBUS_RESPONSE) {
        return;
    }

    // Serial.print("EVMeter Response\n");
    // Packet from EV electric meter
    if (MB.Register == EMConfig[evMeter].ERegister) {
        // Energy measurement
        evMeterEnergy = receiveEnergyMeasurement(MB.Data, evMeter);
        if (evMeterResetKwh == EVMETER_RESET_KWH_POWERUP) {
            // At powerup, set EnergyEV to kwh meter value
            evMeterEnergyAtStart = evMeterEnergy;
        }

        // Calculate Energy
        energyCharged = evMeterEnergy - evMeterEnergyAtStart;
    } else if (MB.Register == EMConfig[evMeter].PRegister) {
        // Power measurement
        powerMeasured = receivePowerMeasurement(MB.Data, evMeter);
    }
}

// Monitor PV Meter responses, and update PV current measurements
// Does not send any data back.
//
ModbusMessage EVSEModbus::onModbusPVMeterResponse(ModbusMessage msg) {
    evseModbus.modbusPVMeterResponseHandler(msg);
    // As this is a response to an earlier request, do not send response.
    return NIL_RESPONSE;
}

void EVSEModbus::modbusPVMeterResponseHandler(ModbusMessage request) {
    ModbusDecode((uint8_t*)request.data(), request.size());

    if (MB.Type == MODBUS_RESPONSE) {
        return;
    }

    //        Serial.print("PVMeter Response\n");
    if (pvMeter != PV_METER_DISABLED && MB.Address == pvMeterAddress && MB.Register == EMConfig[pvMeter].IRegister) {
        // packet from PV electric meter
        receiveCurrentMeasurement(MB.Data, pvMeter, PV);
    }
}

//
// Monitor Mains Meter responses, and update Irms values
// Does not send any data back.
ModbusMessage EVSEModbus::onModbusMainsMeterResponse(ModbusMessage msg) {
    evseModbus.modbusMainsMeterResponseHandler(msg);

    // As this is a response to an earlier request, do not send response.
    return NIL_RESPONSE;
}

void EVSEModbus::modbusMainsMeterResponseHandler(ModbusMessage msg) {
    uint8_t x;

    ModbusDecode((uint8_t*)msg.data(), msg.size());

    // process only Responses, as otherwise MB.Data is unitialized, and it will throw an exception
    if (MB.Register == EMConfig[mainsMeter].IRegister && MB.Type == MODBUS_RESPONSE) {
        // Serial.print("Mains Meter Response\n");
        x = receiveCurrentMeasurement(MB.Data, mainsMeter, CM);
        // only reset timeout when data is ok, and Master/Disabled
        if (x && amIMasterOrDisabled()) {
            // Clear communication error, if present
            if (evseController.errorFlags & ERROR_FLAG_CT_NOCOMM) {
                evseController.errorFlags &= ~ERROR_FLAG_CT_NOCOMM;
            }
            lastCTResponseMillis = millis();
        }

        // Calculate Isum (for nodes and master)
        Isum = 0;
        for (x = 0; x < 3; x++) {
            // Calculate difference of Mains and PV electric meter
            if (pvMeter != PV_METER_DISABLED) {
                // CurrentMeter and PV resolution are 1mA
                CM[x] = CM[x] - PV[x];
            }

            // reduce resolution of Irms to 100mA
            evseController.Irms[x] = (signed int)(CM[x] / 100);
            // Isum has a resolution of 100mA
            Isum += evseController.Irms[x];
        }
    }
}

// Request handler for modbus messages addressed to -this- Node/Slave EVSE.
// Sends response back to Master
ModbusMessage EVSEModbus::onModbusNodeRequest(ModbusMessage request) {
    return evseModbus.modbusNodeRequestHandler(request);
}

ModbusMessage EVSEModbus::modbusNodeRequestHandler(ModbusMessage request) {
    ModbusMessage response;  // response message to be sent back
    uint8_t i, OK = 0;
    uint16_t value, values[MODBUS_MAX_REGISTER_READ];

    // Check if the call is for our current ServerID, or maybe for an old ServerID?
    if (getLoadBl() != request.getServerID()) {
        return NIL_RESPONSE;
    }

    ModbusDecode((uint8_t*)request.data(), request.size());
    uint8_t ItemID = mapModbusRegister2MenuItemID();

    switch (MB.Function) {
        case MODBUS_FUNCTION_READ_HOLDING_REGISTER:
        case MODBUS_FUNCTION_READ_INPUT_REGISTER:
            if (ItemID) {
                response.add(MB.Address, MB.Function, (uint8_t)(MB.RegisterCount * 2));

                for (i = 0; i < MB.RegisterCount; i++) {
                    // TODO SCF i don't like calling menu in modbus (to get, set and broadcast changes to nodes)
                    values[i] = evseMenu.getMenuItemValue(ItemID + i);
                    response.add(values[i]);
                }
            } else {
                response.setError(MB.Address, MB.Function, ILLEGAL_DATA_ADDRESS);
            }
            break;

        case MODBUS_FUNCTION_WRITE_SINGLE_REGISTER:
            if (ItemID) {
                OK = evseMenu.setMenuItemValue(ItemID, MB.Value);
            }

            if (OK && ItemID < NODE_STATUS_STATE) {
                evseMenu.updateSettings();
            }

            if (MB.Address != BROADCAST_ADR || isLoadBalancerDisabled()) {
                if (!ItemID) {
                    response.setError(MB.Address, MB.Function, ILLEGAL_DATA_ADDRESS);
                } else if (!OK) {
                    response.setError(MB.Address, MB.Function, ILLEGAL_DATA_VALUE);
                } else {
                    return ECHO_RESPONSE;
                }
            }
            break;

        case MODBUS_FUNCTION_WRITE_MULTIPLE_REGISTER:
            if (ItemID) {
                for (i = 0; i < MB.RegisterCount; i++) {
                    value = (MB.Data[i * 2] << 8) | MB.Data[(i * 2) + 1];
                    OK += evseMenu.setMenuItemValue(ItemID + i, value);
                }
            }

            if (OK && ItemID < NODE_STATUS_STATE) {
                evseMenu.updateSettings();
            }

            if (MB.Address != BROADCAST_ADR || isLoadBalancerDisabled()) {
                if (!ItemID) {
                    response.setError(MB.Address, MB.Function, ILLEGAL_DATA_ADDRESS);
                } else if (!OK) {
                    response.setError(MB.Address, MB.Function, ILLEGAL_DATA_VALUE);
                } else {
                    response.add(MB.Address, MB.Function, (uint16_t)MB.Register, (uint16_t)OK);
                }
            }
            break;
    }

    return response;
}

// The Node/Server receives a broadcast message from the Master
// Does not send any data back.
ModbusMessage EVSEModbus::onModbusServerbroadcast(ModbusMessage msg) {
    evseModbus.modbusBbroadcastHandler(msg);

    // As it is a broadcast message, do not send response.
    return NIL_RESPONSE;
}

void EVSEModbus::modbusBbroadcastHandler(ModbusMessage msg) {
    uint8_t ItemID, i, OK = 0;
    uint16_t value;

    ModbusDecode((uint8_t*)msg.data(), msg.size());
    ItemID = mapModbusRegister2MenuItemID();

    if (MB.Type == MODBUS_REQUEST) {
        // Broadcast or addressed to this device
        switch (MB.Function) {
            // FC 03 and 04 are not possible with broadcast messages.
            case MODBUS_FUNCTION_WRITE_SINGLE_REGISTER:
                if (ItemID) {
                    OK = evseMenu.setMenuItemValue(ItemID, MB.Value);
                }

                if (OK && ItemID < NODE_STATUS_STATE)
                    evseMenu.updateSettings();
                break;

            case MODBUS_FUNCTION_WRITE_MULTIPLE_REGISTER:
                // Message for Node(s)
                if (MB.Register == MODBUS_REG_BALANCECURRENTS && amIWorkerNode()) {
                    balancedCurrent[0] = (MB.Data[(getLoadBl() - 1) * 2] << 8) | MB.Data[(getLoadBl() - 1) * 2 + 1];
                    if (balancedCurrent[0] == 0 && evseController.state == STATE_C_CHARGING) {
                        // tell EV to stop charging if charge current is zero
                        evseController.setState(STATE_C1_CHARGING_NO_POWER);
                    } else if ((evseController.state == STATE_B_VEHICLE_DETECTED) || (evseController.state == STATE_C_CHARGING)) {
                        // Set charge current, and PWM output
                        evseController.setCurrent(balancedCurrent[0]);
                    }
                    lastCTResponseMillis = millis();
                } else {
                    if (ItemID) {
                        for (i = 0; i < MB.RegisterCount; i++) {
                            value = (MB.Data[i * 2] << 8) | MB.Data[(i * 2) + 1];
                            OK += evseMenu.setMenuItemValue(ItemID + i, value);
                        }
                    }

                    if (OK && ItemID < NODE_STATUS_STATE) {
                        evseMenu.updateSettings();
                    }
                }
                break;
        }
    }
}

/**
 * Master requests Node configuration over modbus
 * Master -> Node
 *
 * @param uint8_t NodeNr (1-7)
 */
void EVSEModbus::requestNodeConfig(uint8_t NodeNr) {
    ModbusReadInputRequest(NodeNr + 1u, 4, MODBUS_FUNCTION_NODE_CONFIG_REGISTER, 2);
}

/**
 * Master requests Node status over modbus
 * Master -> Node
 *
 * @param uint8_t NodeNr (1-7)
 */
void EVSEModbus::requestNodeStatus(uint8_t NodeNr) {
    Node[NodeNr].Online = false;
    ModbusReadInputRequest(NodeNr + 1u, 4, MODBUS_FUNCTION_NODE_STATUS_REGISTER, 8);
}

/**
 * Master receives Node status over modbus
 * Node -> Master
 *
 * @param uint8_t NodeAdr (1-7)
 */
void EVSEModbus::receiveNodeStatus(uint8_t* buf, uint8_t NodeNr) {
    Node[NodeNr].Online = (isLoadBalancerMaster()) ? true : false;

    //    memcpy(buf, (uint8_t*)&Node[NodeNr], sizeof(struct NodeState));
    // Node State
    balancedState[NodeNr] = buf[1];
    // Node Error status
    balancedError[NodeNr] = buf[3];
    Node[NodeNr].ConfigChanged = buf[13] | Node[NodeNr].ConfigChanged;
    // Node Max ChargeCurrent (0.1A)
    balancedMax[NodeNr] = buf[15] * 10;
    // Serial.printf("ReceivedNode[%u]Status State:%u Error:%u, BalancedMax:%u\n", NodeNr, balancedState[NodeNr], balancedError[NodeNr], BalancedMax[NodeNr] );
}

// Data handler for Master
// Responses from Slaves/Nodes are handled here
void EVSEModbus::modbusOnClientData(ModbusMessage msg, uint32_t token) {
    evseModbus.modbusClientDataHandler(msg, token);
}

// Data handler for Master
// Responses from Slaves/Nodes are handled here
void EVSEModbus::modbusClientDataHandler(ModbusMessage msg, uint32_t token) {
    uint8_t address = msg.getServerID();

    if (address == mainsMeterAddress) {
        // Serial.print("MainsMeter data\n");
        modbusMainsMeterResponseHandler(msg);
        return;
    }

    if (address == evMeterAddress) {
        // Serial.print("EV Meter data\n");
        modbusEVMeterResponseHandler(msg);
        return;
    }

    if (address == pvMeterAddress) {
        // Serial.print("PV Meter data\n");
        modbusPVMeterResponseHandler(msg);
        // Only responses to FC 03/04 are handled here. FC 06/10 response is only a acknowledge.
        return;
    }

    ModbusDecode((uint8_t*)msg.data(), msg.size());

    if (MB.Address > 1 && MB.Address <= NR_EVSES && (MB.Function == 03 || MB.Function == 04)) {
        // Packet from Node EVSE
        if (MB.Register == MODBUS_FUNCTION_NODE_STATUS_REGISTER) {
            // Node status
            //    Serial.print("Node Status received\n");
            receiveNodeStatus(evseModbus.MB.Data, MB.Address - 1u);
        } else if (MB.Register == MODBUS_FUNCTION_NODE_CONFIG_REGISTER) {
            // Node EV meter settings
            //    Serial.print("Node EV Meter settings received\n");
            receiveNodeConfig(MB.Data, MB.Address - 1u);
        }
    }
}

void EVSEModbus::modbusWorkflowNormalMode() {
    // --------------------------------------------------------------
    // ------------- Cluster of EVs in Normal mode ------------------
    // --------------------------------------------------------------
    switch (workflowModbusRequest) {
        // ------------- Retrieve data from EVs nodes -------------------
        case 6:   // Node 1
        case 7:   // Node 2
        case 8:   // Node 3
        case 9:   // Node 4
        case 10:  // Node 5
        case 11:  // Node 6
        case 12:  // Node 7
            if (!isLoadBalancerMaster()) {
                // Skip to process data
                workflowModbusRequest = 13;
                break;
            }

            // Master, request Node 1-8 status
            // NR_EVSES = 8 (0 - 7), substract master (pos 0) => then 6u
            requestNodeStatus(workflowModbusRequest - 6u);
            workflowModbusRequest++;
            break;

        // ------------- Process data from EVs nodes -------------------
        case 13:  // Node 1
        case 14:  // Node 2
        case 15:  // Node 3
        case 16:  // Node 4
        case 17:  // Node 5
        case 18:  // Node 6
        case 19:  // Node 7
            if (!isLoadBalancerMaster()) {
                // Skip but give some time for CT data to be sent/receive over bus
                workflowModbusRequest++;
                break;
            }

            // Master, process Node 1-8 status
            // NR_EVSES = 8 (0 - 7), substract master (pos 0) => then 6u plus 7 nodes => 13u
            processAllNodeStates(workflowModbusRequest - 13u);
            workflowModbusRequest++;
            break;

        default:
            onCTDataReceived();
            workflowModbusRequest = 0;
            // Serial.printf("Task free ram: %u\n", uxTaskGetStackHighWaterMark( NULL ));
    }
}

void EVSEModbus::modbusWorkflowSolarSmartMode() {
    switch (workflowModbusRequest) {
        case 1:
            if (pvMeter == PV_METER_DISABLED) {
                // Skip PV measurement if no PVMeter
                workflowModbusRequest = 3;
                break;
            }

            // PV kwh meter enabled then read it and read MainsMeter
            requestCurrentMeasurement(pvMeter, pvMeterAddress);
            workflowModbusRequest = 2;
            break;

        case 2:
            // Sensorbox or kWh meter that measures -all- currents
            requestCurrentMeasurement(mainsMeter, mainsMeterAddress);
            workflowModbusRequest = 3;
            break;

        case 3:
            // Find next online SmartEVSE
            do {
                workflowPollingEVNodeNumber++;
                if (workflowPollingEVNodeNumber >= NR_EVSES)
                    workflowPollingEVNodeNumber = 0;
            } while (Node[workflowPollingEVNodeNumber].Online == false);

            // Request Configuration if changed
            if (Node[workflowPollingEVNodeNumber].ConfigChanged == 0) {
                // If no config changed skip
                workflowModbusRequest = 5;
                break;
            }

            requestNodeConfig(workflowPollingEVNodeNumber);
            workflowModbusRequest = 4;
            break;

        case 4:
            // EV kWh meter, Energy measurement (total charged kWh)
            // Request Energy if EV meter is configured
            if (Node[workflowPollingEVNodeNumber].EVMeter == 0) {
                // If no meter skip
                workflowModbusRequest = 99;
                break;
            }

            requestEnergyMeasurement(Node[workflowPollingEVNodeNumber].EVMeter, Node[workflowPollingEVNodeNumber].EVAddress);
            workflowModbusRequest = 5;
            break;

        case 5:
            // EV kWh meter, Power measurement (momentary power in Watt)
            // Request Power if EV meter is configured
            if (Node[workflowPollingEVNodeNumber].EVMeter == 0) {
                // If no meter skip
                workflowModbusRequest = 99;
                break;
            }

            requestPowerMeasurement(Node[workflowPollingEVNodeNumber].EVMeter, Node[workflowPollingEVNodeNumber].EVAddress);
            workflowModbusRequest = 99;
            break;

        default:
            onCTDataReceived();
            workflowModbusRequest = 0;
            // Serial.printf("Task free ram: %u\n", uxTaskGetStackHighWaterMark( NULL ));
    }
}

void EVSEModbus::modbusWorkflow() {
    if (evseController.mode == MODE_NORMAL) {
        modbusWorkflowNormalMode();
    } else {
        modbusWorkflowSolarSmartMode();
    }
}


/**
 * Map a Modbus register to an item ID (MENU_xxx or STATUS_xxx)
 *
 * @return uint8_t ItemID
 */
uint8_t EVSEModbus::mapModbusRegister2MenuItemID() {
    uint16_t RegisterStart, ItemStart, Count;

    // Register 0x00*: Status
    if (MB.Register >= MODBUS_EVSE_STATUS_START && MB.Register < (MODBUS_EVSE_STATUS_START + MODBUS_EVSE_STATUS_COUNT)) {
        RegisterStart = MODBUS_EVSE_STATUS_START;
        ItemStart = NODE_STATUS_STATE;
        Count = MODBUS_EVSE_STATUS_COUNT;

        // Register 0x01*: Node specific configuration
    } else if (MB.Register >= MODBUS_EVSE_CONFIG_START && MB.Register < (MODBUS_EVSE_CONFIG_START + MODBUS_EVSE_CONFIG_COUNT)) {
        RegisterStart = MODBUS_EVSE_CONFIG_START;
        ItemStart = MENU_CONFIG;
        Count = MODBUS_EVSE_CONFIG_COUNT;

        // Register 0x02*: System configuration (same on all SmartEVSE in a LoadBalancing setup)
    } else if (MB.Register >= MODBUS_SYS_CONFIG_START && MB.Register < (MODBUS_SYS_CONFIG_START + MODBUS_SYS_CONFIG_COUNT)) {
        RegisterStart = MODBUS_SYS_CONFIG_START;
        ItemStart = MENU_MODE;
        Count = MODBUS_SYS_CONFIG_COUNT;

    } else {
        return 0;
    }

    if (MB.RegisterCount <= (RegisterStart + Count) - MB.Register) {
        return (MB.Register - RegisterStart + ItemStart);
    }

    return 0;
}


void EVSEModbus::readEpromSettings() {
    Preferences preferences;
    if (preferences.begin(PREFS_CLUSTER_NAMESPACE, true) != true) {
        EVSELogger::error("Unable to open preferences for EVSEModbus");
        return;
    }

    bool firstRun = true;
    if (preferences.isKey(PREFS_LOADBL_KEY)) {
        firstRun = false;

        LoadBl = preferences.getUChar(PREFS_LOADBL_KEY, LOAD_BALANCER_DISABLED);
        evMeter = preferences.getUChar(PREFS_EVMETER_KEY, EV_METER_DISABLED);
        evMeterAddress = preferences.getUChar(PREFS_EVMETERADDRESS_KEY, EV_METER_ADDRESS);
        pvMeter = preferences.getUChar(PREFS_PVMETER_KEY, PV_METER_DISABLED);
        pvMeterAddress = preferences.getUChar(PREFS_PVMADDRESS_KEY, PV_METER_ADDRESS);
        grid = preferences.getUChar(PREFS_GRID_KEY, GRID);
        mainsMeterAddress = preferences.getUChar(PREFS_MAINSMETERADDRESS_KEY, MAINS_METER_ADDRESS);
        mainsMeter = preferences.getUChar(PREFS_MAINSMETER_KEY, MM_SENSORBOX);
        mainsMeterMeasure = preferences.getUChar(PREFS_MAINSMETERMEASURE_KEY, MAINS_METER_MEASURE);
        maxCircuit = preferences.getUShort(PREFS_MAXCIRCUIT_KEY, MAX_CIRCUIT);

        EMConfig[MM_CUSTOM].Endianness = preferences.getUChar("EMEndianness", EMCUSTOM_ENDIANESS);
        EMConfig[MM_CUSTOM].IRegister = preferences.getUShort("EMIRegister", EMCUSTOM_IREGISTER);
        EMConfig[MM_CUSTOM].IDivisor = preferences.getUChar("EMIDivisor", EMCUSTOM_IDIVISOR);
        EMConfig[MM_CUSTOM].URegister = preferences.getUShort("EMURegister", EMCUSTOM_UREGISTER);
        EMConfig[MM_CUSTOM].UDivisor = preferences.getUChar("EMUDivisor", EMCUSTOM_UDIVISOR);
        EMConfig[MM_CUSTOM].PRegister = preferences.getUShort("EMPRegister", EMCUSTOM_PREGISTER);
        EMConfig[MM_CUSTOM].PDivisor = preferences.getUChar("EMPDivisor", EMCUSTOM_PDIVISOR);
        EMConfig[MM_CUSTOM].ERegister = preferences.getUShort("EMERegister", EMCUSTOM_EREGISTER);
        EMConfig[MM_CUSTOM].EDivisor = preferences.getUChar("EMEDivisor", EMCUSTOM_EDIVISOR);
        EMConfig[MM_CUSTOM].DataType = (mb_datatype)preferences.getUChar("EMDataType", EMCUSTOM_DATATYPE);
        EMConfig[MM_CUSTOM].Function = preferences.getUChar("EMFunction", EMCUSTOM_FUNCTION);
    }
    preferences.end();

    if (firstRun) {
        LoadBl = LOAD_BALANCER_DISABLED;
        evMeter = EV_METER_DISABLED;
        evMeterAddress = EV_METER_ADDRESS;
        pvMeter = PV_METER_DISABLED;
        pvMeterAddress = PV_METER_ADDRESS;
        grid = GRID;
        mainsMeterAddress = MAINS_METER_ADDRESS;
        mainsMeter = MM_SENSORBOX;
        mainsMeterMeasure = MAINS_METER_MEASURE;
        maxCircuit = MAX_CIRCUIT;

        EMConfig[MM_CUSTOM].Endianness = EMCUSTOM_ENDIANESS;
        EMConfig[MM_CUSTOM].IRegister = EMCUSTOM_IREGISTER;
        EMConfig[MM_CUSTOM].IDivisor = EMCUSTOM_IDIVISOR;
        EMConfig[MM_CUSTOM].URegister = EMCUSTOM_UREGISTER;
        EMConfig[MM_CUSTOM].UDivisor = EMCUSTOM_UDIVISOR;
        EMConfig[MM_CUSTOM].PRegister = EMCUSTOM_PREGISTER;
        EMConfig[MM_CUSTOM].PDivisor = EMCUSTOM_PDIVISOR;
        EMConfig[MM_CUSTOM].ERegister = EMCUSTOM_EREGISTER;
        EMConfig[MM_CUSTOM].EDivisor = EMCUSTOM_EDIVISOR;
        EMConfig[MM_CUSTOM].DataType = (mb_datatype)EMCUSTOM_DATATYPE;
        EMConfig[MM_CUSTOM].Function = EMCUSTOM_FUNCTION;

        writeEpromSettings();
    }
}

void EVSEModbus::validateSettings() {
    // Sensorbox v2 has always address 0x0A
    if (mainsMeter == MM_SENSORBOX) {
        mainsMeterAddress = MM_SENSORBOX_ADDRESS;
    }

    // Disable PV reception if not configured
    if (mainsMeterMeasure == MAINS_METER_MEASURE) {
        pvMeter = PV_METER_DISABLED;
    }

    Node[0].EVMeter = evMeter;
    Node[0].EVAddress = evMeterAddress;

    // Default to modbus input registers
    if (EMConfig[MM_CUSTOM].Function != 3) {
        EMConfig[MM_CUSTOM].Function = 4;
    }

    // Backward compatibility < 2.20
    if (EMConfig[MM_CUSTOM].IRegister == 8 || EMConfig[MM_CUSTOM].URegister == 8 || EMConfig[MM_CUSTOM].PRegister == 8 || EMConfig[MM_CUSTOM].ERegister == 8) {
        EMConfig[MM_CUSTOM].DataType = MB_DATATYPE_FLOAT32;
        EMConfig[MM_CUSTOM].IRegister = 0;
        EMConfig[MM_CUSTOM].URegister = 0;
        EMConfig[MM_CUSTOM].PRegister = 0;
        EMConfig[MM_CUSTOM].ERegister = 0;
    }
}

void EVSEModbus::writeEpromSettings() {
    validateSettings();

    Preferences preferences;
    if (preferences.begin(PREFS_CLUSTER_NAMESPACE, false) != true) {
        EVSELogger::error("Unable to write preferences for EVSEModbus");
        return;
    }

    preferences.putUChar(PREFS_LOADBL_KEY, LoadBl);
    preferences.putUChar(PREFS_EVMETER_KEY, evMeter);
    preferences.putUChar(PREFS_EVMETERADDRESS_KEY, evMeterAddress);
    preferences.putUChar(PREFS_PVMETER_KEY, pvMeter);
    preferences.putUChar(PREFS_PVMADDRESS_KEY, pvMeterAddress);
    preferences.putUChar(PREFS_GRID_KEY, grid);
    preferences.putUChar(PREFS_MAINSMETERADDRESS_KEY, mainsMeterAddress);
    preferences.putUChar(PREFS_MAINSMETERMEASURE_KEY, mainsMeterMeasure);
    preferences.putUChar(PREFS_MAINSMETER_KEY, mainsMeter);
    preferences.putUShort(PREFS_MAXCIRCUIT_KEY, maxCircuit);

    preferences.putUChar("EMEndianness", EMConfig[MM_CUSTOM].Endianness);
    preferences.putUShort("EMIRegister", EMConfig[MM_CUSTOM].IRegister);
    preferences.putUChar("EMIDivisor", EMConfig[MM_CUSTOM].IDivisor);
    preferences.putUShort("EMURegister", EMConfig[MM_CUSTOM].URegister);
    preferences.putUChar("EMUDivisor", EMConfig[MM_CUSTOM].UDivisor);
    preferences.putUShort("EMPRegister", EMConfig[MM_CUSTOM].PRegister);
    preferences.putUChar("EMPDivisor", EMConfig[MM_CUSTOM].PDivisor);
    preferences.putUShort("EMERegister", EMConfig[MM_CUSTOM].ERegister);
    preferences.putUChar("EMEDivisor", EMConfig[MM_CUSTOM].EDivisor);
    preferences.putUChar("EMDataType", EMConfig[MM_CUSTOM].DataType);
    preferences.putUChar("EMFunction", EMConfig[MM_CUSTOM].Function);

    preferences.end();
}

void EVSEModbus::updateSettings() {
    writeEpromSettings();

    if (!isLoadBalancerMaster()) {
        return;
    }

    // Broadcast settings to other controllers
    uint16_t values[MODBUS_SYS_CONFIG_COUNT];
    for (uint8_t i = 0; i < MODBUS_SYS_CONFIG_COUNT; i++) {
        values[i] = evseMenu.getMenuItemValue(MENU_MODE + i);
    }
    broadcastSysConfigToNodes(values);
}

void EVSEModbus::resetSettings() {
    readEpromSettings();
}

void EVSEModbus::configureModbusMode(uint8_t newmode) {
    MBserver = new ModbusServerRTU(Serial1, 2000, PIN_RS485_DIR);
    MBclient = new ModbusClientRTU(Serial1, PIN_RS485_DIR);

    char buffer[50];
    sprintf(buffer, "Changing LoadBL from %u to %u", LoadBl, newmode);
    EVSELogger::info(buffer);

    // Start mode or convert master to worker
    if (amIMasterOrDisabled() && (newmode == MODBUS_START_MODE || newmode > 1)) {
        EVSELogger::info("Setup Modbus as Master/Client, stop Server/Node handler");

        // Setup Modbus workers as Master. Stop Node background task (if active)
        if (newmode != MODBUS_START_MODE) {
            LoadBl = newmode;
            MBserver->stop();
        }
        // EVSELogger::debug("task free ram: %u", uxTaskGetStackHighWaterMark(NULL));

        MBclient->setTimeout(100);
        MBclient->onDataHandler(&modbusOnClientData);
        MBclient->onErrorHandler(&modbusOnClientError);

        // Start ModbusRTU Master backgroud task
        MBclient->begin();
        return;
    }

    // Start mode or convert worker to masters
    if (amIWorkerNode() && (newmode == MODBUS_START_MODE || newmode < 2)) {
        // Setup Modbus workers for Node
        EVSELogger::info("Setup MBserver/Node workers, end Master/Client");

        if (newmode != MODBUS_START_MODE) {
            LoadBl = newmode;
            // Stop Master background task (if active)
            MBclient->end();
        }
        // EVSELogger::debug("task free ram: %u", uxTaskGetStackHighWaterMark(NULL));

        // Register worker. at serverID 'LoadBl', all function codes
        MBserver->registerWorker(LoadBl, ANY_FUNCTION_CODE, &onModbusNodeRequest);
        // Also add handler for all broadcast messages from Master.
        MBserver->registerWorker(BROADCAST_ADR, ANY_FUNCTION_CODE, &onModbusServerbroadcast);

        if (mainsMeter != MAINS_METER_DISABLED) {
            MBserver->registerWorker(mainsMeterAddress, ANY_FUNCTION_CODE, &onModbusMainsMeterResponse);
        }

        if (evMeter != EV_METER_DISABLED) {
            MBserver->registerWorker(evMeterAddress, ANY_FUNCTION_CODE, &onModbusEVMeterResponse);
        }

        if (pvMeter != PV_METER_DISABLED) {
            MBserver->registerWorker(pvMeterAddress, ANY_FUNCTION_CODE, &onModbusPVMeterResponse);
        }

        // Start ModbusRTU Node background task
        MBserver->start();
        return;
    }

    // Change worker id
    if ((newmode != MODBUS_START_MODE) && newmode > 1) {
        // Register worker. at serverID 'LoadBl', all function codes
        sprintf(buffer, "Registering new LoadBl worker at id %u", newmode);
        EVSELogger::info(buffer);
        LoadBl = newmode;
        MBserver->registerWorker(newmode, ANY_FUNCTION_CODE, &onModbusNodeRequest);
    }
}

void EVSEModbus::setup() {
    readEpromSettings();

    configureModbusMode(MODBUS_START_MODE);
}

void EVSEModbus::loop() {
    // ctCommTimeout if CT current measurement takes > 10 secs
    if ((millis() - lastCTResponseMillis) > DEFAULT_CT_COMM_TIMEOUT) {
        onCTCommunicationLost();
    }

    // TODO SCF refactor modbus workflow and switch-case flow
    //  Every two seconds request measurement data from sensorbox/kwh meters.
    if ((millis() - lastCTRequestMillis) >= DEFAULT_CT_REQUEST_PERIOD) {
        lastCTRequestMillis = millis();
        // EVSE will process broadcast every 100ms. While requesting for measures, EVSE will process broadcast every 100ms. Default EVSE or master in LB mode,
        // send broadcast to Nodes
        if (amIMasterOrDisabled()) {
            resetCTDataMeasured();

            workflowModbusRequest = evseController.mode == MODE_NORMAL ? 6 : 1;
        }
    }

    // Every 2 seconds, request measurements from modbus meters
    if (workflowModbusRequest > 0) {
        modbusWorkflow();
    }

    // Every 1 second
    /*// Charge timer
    for (uint8_t x = 0; x < NR_EVSES; x++) {
        if (balancedState[x] == STATE_C_CHARGING) {
            Node[x].Timer++;
        }
    }*/
}

// ########################## Modbus helper functions ##########################

/**
 * Send single value over modbus
 *
 * @param uint8_t address
 * @param uint8_t function
 * @param uint16_t register
 * @param uint16_t data
 */
void EVSEModbus::ModbusSend8(uint8_t address, uint8_t function, uint16_t reg, uint16_t data) {
    // 0x12345678 is a token to keep track of modbus requests/responses. currently unused.
    MBclient->addRequest(0x12345678, address, function, reg, data);
}

/**
 * Combine Bytes received over modbus
 *
 * @param pointer to var
 * @param pointer to buf
 * @param uint8_t pos
 * @param uint8_t endianness:\n
 *        0: low byte first, low word first (little endian)\n
 *        1: low byte first, high word first\n
 *        2: high byte first, low word first\n
 *        3: high byte first, high word first (big endian)
 * @param MBDataType dataType: used to determine how many bytes should be combined
 */
void EVSEModbus::combineBytes(void* var, uint8_t* buf, uint8_t pos, uint8_t endianness, MBDataType dataType) {
    char* pBytes;
    pBytes = (char*)var;

    // ESP32 is little endian
    switch (endianness) {
        case ENDIANESS_LBF_LWF:  // low byte first, low word first (little endian)
            *pBytes++ = (uint8_t)buf[pos + 0];
            *pBytes++ = (uint8_t)buf[pos + 1];
            if (dataType != MB_DATATYPE_INT16) {
                *pBytes++ = (uint8_t)buf[pos + 2];
                *pBytes = (uint8_t)buf[pos + 3];
            }
            break;
            
        case ENDIANESS_LBF_HWF:  // low byte first, high word first
            if (dataType != MB_DATATYPE_INT16) {
                *pBytes++ = (uint8_t)buf[pos + 2];
                *pBytes++ = (uint8_t)buf[pos + 3];
            }
            *pBytes++ = (uint8_t)buf[pos + 0];
            *pBytes = (uint8_t)buf[pos + 1];
            break;

        case ENDIANESS_HBF_LWF:  // high byte first, low word first
            *pBytes++ = (uint8_t)buf[pos + 1];
            *pBytes++ = (uint8_t)buf[pos + 0];
            if (dataType != MB_DATATYPE_INT16) {
                *pBytes++ = (uint8_t)buf[pos + 3];
                *pBytes = (uint8_t)buf[pos + 2];
            }
            break;

        case ENDIANESS_HBF_HWF:  // high byte first, high word first (big endian)
            if (dataType != MB_DATATYPE_INT16) {
                *pBytes++ = (uint8_t)buf[pos + 3];
                *pBytes++ = (uint8_t)buf[pos + 2];
            }
            *pBytes++ = (uint8_t)buf[pos + 1];
            *pBytes = (uint8_t)buf[pos + 0];
            break;
    }
}

// ########################### Modbus main functions ###########################

/**
 * Request read holding (FC=3) or read input register (FC=04) to a device over modbus
 *
 * @param uint8_t address
 * @param uint8_t function
 * @param uint16_t register
 * @param uint16_t quantity
 */
void EVSEModbus::ModbusReadInputRequest(uint8_t address, uint8_t function, uint16_t reg, uint16_t quantity) {
    MB.RequestAddress = address;
    MB.RequestFunction = function;
    MB.RequestRegister = reg;
    ModbusSend8(address, function, reg, quantity);
}

/**
 * Request write single register (FC=06) to a device over modbus
 *
 * @param uint8_t address
 * @param uint16_t register
 * @param uint16_t value
 */
void EVSEModbus::ModbusWriteSingleRequest(uint8_t address, uint16_t reg, uint16_t value) {
    MB.RequestAddress = address;
    MB.RequestFunction = MODBUS_FUNCTION_WRITE_SINGLE_REGISTER;
    MB.RequestRegister = reg;
    ModbusSend8(address, MODBUS_FUNCTION_WRITE_SINGLE_REGISTER, reg, value);
}

/**
 * Request write multiple register (FC=16) to a device over modbus
 *
 * @param uint8_t address
 * @param uint16_t register
 * @param uint8_t pointer to data
 * @param uint8_t count of data
 */
void EVSEModbus::ModbusWriteMultipleRequest(uint8_t address, uint16_t reg, uint16_t* values, uint8_t count) {
    MB.RequestAddress = address;
    MB.RequestFunction = MODBUS_FUNCTION_WRITE_MULTIPLE_REGISTER;
    MB.RequestRegister = reg;
    // 0x12345678 is a token to keep track of modbus requests/responses. currently unused.
    MBclient->addRequest(0x12345678, address, MODBUS_FUNCTION_WRITE_MULTIPLE_REGISTER, reg, (uint16_t)count, count * 2u, values);
}

/**
 * Decode received modbus packet
 *
 * @param uint8_t pointer to buffer
 * @param uint8_t length of buffer
 */
void EVSEModbus::ModbusDecode(uint8_t* buf, uint8_t len) {
    // Clear old values
    MB.Address = 0;
    MB.Function = 0;
    MB.Register = 0;
    MB.RegisterCount = 0;
    MB.Value = 0;
    MB.DataLength = 0;
    MB.Type = MODBUS_INVALID;
    MB.Exception = 0;

    // Modbus error packets length is 5 bytes
    if (len == 3) {
        MB.Type = MODBUS_EXCEPTION;
        // Modbus device address
        MB.Address = buf[0];
        // Modbus function
        MB.Function = buf[1];
        // Modbus Exception code
        MB.Exception = buf[2];
        // Modbus data packets minimum length is 8 bytes
    } else if (len >= 6) {
        // Modbus device address
        MB.Address = buf[0];
        // Modbus function
        MB.Function = buf[1];

        switch (MB.Function) {
            case MODBUS_FUNCTION_READ_HOLDING_REGISTER:
            case MODBUS_FUNCTION_READ_INPUT_REGISTER:
                if (len == 6) {
                    // request packet
                    MB.Type = MODBUS_REQUEST;
                    // Modbus register
                    MB.Register = (uint16_t)(buf[2] << 8) | buf[3];
                    // Modbus register count
                    MB.RegisterCount = (uint16_t)(buf[4] << 8) | buf[5];
                } else {
                    // Modbus datacount
                    MB.DataLength = buf[2];
                    if (MB.DataLength == len - 3) {
                        // packet length OK
                        // response packet
                        MB.Type = MODBUS_RESPONSE;
                    }
                }
                break;

            case MODBUS_FUNCTION_WRITE_SINGLE_REGISTER:
                if (len == 6) {
                    // request and response packet are the same
                    MB.Type = MODBUS_OK;
                    // Modbus register
                    MB.Register = (uint16_t)(buf[2] << 8) | buf[3];
                    // Modbus register count
                    MB.RegisterCount = 1;
                    // value
                    MB.Value = (uint16_t)(buf[4] << 8) | buf[5];
                }
                break;

            case MODBUS_FUNCTION_WRITE_MULTIPLE_REGISTER:
                // Modbus register
                MB.Register = (uint16_t)(buf[2] << 8) | buf[3];
                // Modbus register count
                MB.RegisterCount = (uint16_t)(buf[4] << 8) | buf[5];
                if (len == 6) {
                    // response packet
                    MB.Type = MODBUS_RESPONSE;
                } else {
                    // Modbus datacount
                    MB.DataLength = buf[6];
                    if (MB.DataLength == len - 7) {
                        // packet length OK
                        // request packet
                        MB.Type = MODBUS_REQUEST;
                    }
                }
                break;
        }

        // MB.Data
        if (MB.Type && MB.DataLength) {
            // Set pointer to Data
            MB.Data = buf;
            // Modbus data is always at the end ahead the checksum
            MB.Data = MB.Data + (len - MB.DataLength);
        }

        // Request - Response check
        switch (MB.Type) {
            case MODBUS_REQUEST:
                MB.RequestAddress = MB.Address;
                MB.RequestFunction = MB.Function;
                MB.RequestRegister = MB.Register;
                break;

            case MODBUS_RESPONSE:
                // If address and function identical with last send or received request, it is a valid response
                if (MB.Address == MB.RequestAddress && MB.Function == MB.RequestFunction) {
                    if (MB.Function == MODBUS_FUNCTION_READ_HOLDING_REGISTER || MB.Function == MODBUS_FUNCTION_READ_INPUT_REGISTER)
                        MB.Register = MB.RequestRegister;
                }
                MB.RequestAddress = 0;
                MB.RequestFunction = 0;
                MB.RequestRegister = 0;
                break;

            case MODBUS_OK:
                // If address and function identical with last send or received request, it is a valid response
                if (MB.Address == MB.RequestAddress && MB.Function == MB.RequestFunction &&
                    MB.Address != BROADCAST_ADR) {
                    MB.Type = MODBUS_RESPONSE;
                    MB.RequestAddress = 0;
                    MB.RequestFunction = 0;
                    MB.RequestRegister = 0;
                } else {
                    MB.Type = MODBUS_REQUEST;
                    MB.RequestAddress = MB.Address;
                    MB.RequestFunction = MB.Function;
                    MB.RequestRegister = MB.Register;
                }
                break;
        }
    }
}

// ########################### EVSE modbus functions ###########################

/**
 * Send measurement request over modbus
 *
 * @param uint8_t Meter
 * @param uint8_t Address
 * @param uint16_t Register
 * @param uint8_t Count
 */
void EVSEModbus::requestMeasurement(uint8_t Meter, uint8_t Address, uint16_t Register, uint8_t Count) {
    ModbusReadInputRequest(Address, EMConfig[Meter].Function, Register,
                           (EMConfig[Meter].DataType == MB_DATATYPE_INT16 ? Count : (Count * 2u)));
}

/**
 * Decode measurement value
 *
 * @param pointer to buf
 * @param uint8_t Count
 * @param uint8_t Endianness
 * @param MBDataType dataType
 * @param signed char Divisor
 * @return signed int Measurement
 */
signed int EVSEModbus::receiveMeasurement(uint8_t* buf, uint8_t Count, uint8_t Endianness, MBDataType dataType, signed char Divisor) {
    float dCombined;
    signed int lCombined;

    if (dataType == MB_DATATYPE_FLOAT32) {
        combineBytes(&dCombined, buf, Count * (dataType == MB_DATATYPE_INT16 ? 2u : 4u), Endianness, dataType);
        if (Divisor >= 0) {
            lCombined = (signed int)(dCombined / (signed int)pow_10[(unsigned)Divisor]);
        } else {
            lCombined = (signed int)(dCombined * (signed int)pow_10[(unsigned)-Divisor]);
        }
    } else {
        combineBytes(&lCombined, buf, Count * (dataType == MB_DATATYPE_INT16 ? 2u : 4u), Endianness, dataType);
        if (dataType == MB_DATATYPE_INT16) {
            lCombined = (signed int)((int16_t)lCombined); /* sign extend 16bit into 32bit */
        }
        if (Divisor >= 0) {
            lCombined = lCombined / (signed int)pow_10[(unsigned)Divisor];
        } else {
            lCombined = lCombined * (signed int)pow_10[(unsigned)-Divisor];
        }
    }

    return lCombined;
}

/**
 * Send Energy measurement request over modbus
 *
 * @param uint8_t Meter
 * @param uint8_t Address
 */
void EVSEModbus::requestEnergyMeasurement(uint8_t Meter, uint8_t Address) {
    switch (Meter) {
        case MM_SOLAREDGE:
            // Note:
            // - SolarEdge uses 16-bit values, except for this measurement it uses 32bit int format
            // - MM_SOLAREDGE should not be used for EV Energy Measurements
            ModbusReadInputRequest(Address, EMConfig[Meter].Function, EMConfig[Meter].ERegister, 2);
            break;

        default:
            requestMeasurement(Meter, Address, EMConfig[Meter].ERegister, 1);
            break;
    }
}

/**
 * Read energy measurement from modbus
 *
 * @param pointer to buf
 * @param uint8_t Meter
 * @return signed int Energy (Wh)
 */
signed int EVSEModbus::receiveEnergyMeasurement(uint8_t* buf, uint8_t Meter) {
    switch (Meter) {
        case MM_SOLAREDGE:
            // Note:
            // - SolarEdge uses 16-bit values, except for this measurement it uses 32bit int format
            // - MM_SOLAREDGE should not be used for EV Energy Measurements
            return receiveMeasurement(buf, 0, EMConfig[Meter].Endianness, MB_DATATYPE_INT32, EMConfig[Meter].EDivisor - 3);

        default:
            return receiveMeasurement(buf, 0, EMConfig[Meter].Endianness, EMConfig[Meter].DataType,
                                      EMConfig[Meter].EDivisor - 3);
    }
}

/**
 * Send Power measurement request over modbus
 *
 * @param uint8_t Meter
 * @param uint8_t Address
 */
void EVSEModbus::requestPowerMeasurement(uint8_t Meter, uint8_t Address) {
    requestMeasurement(Meter, Address, EMConfig[Meter].PRegister, 1);
}

/**
 * Read Power measurement from modbus
 *
 * @param pointer to buf
 * @param uint8_t Meter
 * @return signed int Power (W)
 */
signed int EVSEModbus::receivePowerMeasurement(uint8_t* buf, uint8_t Meter) {
    switch (Meter) {
        case MM_SOLAREDGE: {
            // Note:
            // - SolarEdge uses 16-bit values, with a extra 16-bit scaling factor
            // - MM_SOLAREDGE should not be used for EV power measurements, only PV power measurements are supported
            int scalingFactor = -(int)receiveMeasurement(buf, 1, EMConfig[Meter].Endianness, EMConfig[Meter].DataType, 0);
            return receiveMeasurement(buf, 0, EMConfig[Meter].Endianness, EMConfig[Meter].DataType, scalingFactor);
        }
        default:
            return receiveMeasurement(buf, 0, EMConfig[Meter].Endianness, EMConfig[Meter].DataType, EMConfig[Meter].PDivisor);
    }
}

/**
 * Send current measurement request over modbus
 *
 * @param uint8_t Meter
 * @param uint8_t Address
 */
void EVSEModbus::requestCurrentMeasurement(uint8_t Meter, uint8_t Address) {
    switch (Meter) {
        case MM_SENSORBOX:
            ModbusReadInputRequest(Address, 4, 0, 20);
            break;

        case MM_EASTRON:
            // Phase 1-3 current: Register 0x06 - 0x0B (unsigned)
            // Phase 1-3 power:   Register 0x0C - 0x11 (signed)
            ModbusReadInputRequest(Address, 4, MODBUS_FUNCTION_WRITE_SINGLE_REGISTER, 12);
            break;

        case MM_ABB:
            // Phase 1-3 current: Register 0x5B0C - 0x5B11 (unsigned)
            // Phase 1-3 power:   Register 0x5B16 - 0x5B1B (signed)
            ModbusReadInputRequest(Address, 3, 0x5B0C, 16);
            break;

        case MM_SOLAREDGE:
            // Read 3 Current values + scaling factor
            ModbusReadInputRequest(Address, EMConfig[Meter].Function, EMConfig[Meter].IRegister, 4);
            break;

        default:
            // Read 3 Current values
            requestMeasurement(Meter, Address, EMConfig[Meter].IRegister, 3);
            break;
    }
}

/**
 * Read current measurement from modbus
 *
 * @param pointer to buf
 * @param uint8_t Meter
 * @param pointer to Current (mA)
 * @return uint8_t error
 */
uint8_t EVSEModbus::receiveCurrentMeasurement(uint8_t* buf, uint8_t Meter, signed int* var) {
    uint8_t x, offset;

    // No CAL option in Menu
    calibrationActive = false;

    switch (Meter) {
        case MM_SENSORBOX:
            // return immediately if the data contains no new P1 or CT measurement
            if (buf[3] == 0) {
                // error!!
                return 0;
            }
            // determine if there is P1 data present, otherwise use CT data
            if (buf[3] & 0x80) {
                // P1 data present
                offset = 4;
            } else {
                // Use CTs
                offset = 7;
            }

            // offset 16 is Smart meter P1 current
            for (x = 0; x < 3; x++) {
                // SmartEVSE works with Amps * 10
                var[x] = receiveMeasurement(buf, offset + x, EMConfig[Meter].Endianness, EMConfig[Meter].DataType,
                                            EMConfig[Meter].IDivisor - 3u);
                // When using CT's , adjust the measurements with calibration value
                if (offset == 7) {
                    if (x == 0) {
                        evseController.Iuncal = abs((var[x] / 10));  // Store uncalibrated CT1 measurement (10mA)
                    }
                    var[x] = var[x] * (signed int)evseController.ICal / ICAL_DEFAULT;
                    // When MaxMains is set to >100A, it's assumed 200A:50ma CT's are used.
                    if (evseController.maxMains > 100) {
                        // Multiply measured currents with 2
                        var[x] = var[x] * 2;
                    }

                    // very small negative currents are shown as zero.
                    if ((var[x] > -1) && (var[x] < 1)) {
                        var[x] = 0;
                    }

                    // Enable CAL option in Menu
                    calibrationActive = true;
                }
            }
            // Set Sensorbox 2 to 3/4 Wire configuration (and phase Rotation) (v2.16)
            if (buf[1] >= 0x10 && offset == 7) {
                // Enable the GRID menu option
                setGridActive(true);
                if ((buf[1] & 0x3) != (grid << 1) && amIMasterOrDisabled()) {
                    ModbusWriteSingleRequest(MM_SENSORBOX_ADDRESS, MODBUS_FUNCTION_SENSORBOX_GRID_REGISTER, grid << 1);
                }
            } else {
                setGridActive(false);
            }
            break;

        case MM_SOLAREDGE: {
            // Need to handle the extra scaling factor
            int scalingFactor = -(int)receiveMeasurement(buf, 3, EMConfig[Meter].Endianness, EMConfig[Meter].DataType, 0);
            // Now decode the three Current values using that scaling factor
            for (x = 0; x < 3; x++) {
                var[x] = receiveMeasurement(buf, x, EMConfig[Meter].Endianness, EMConfig[Meter].DataType, scalingFactor - 3);
            }
            break;
        }

        default:
            for (x = 0; x < 3; x++) {
                var[x] = receiveMeasurement(buf, x, EMConfig[Meter].Endianness, EMConfig[Meter].DataType,
                                            EMConfig[Meter].IDivisor - 3);
            }
            break;
    }

    // Get sign from power measurement on some electric meters
    switch (Meter) {
        case MM_EASTRON:
            for (x = 0; x < 3; x++) {
                if (receiveMeasurement(buf, x + 3u, EMConfig[Meter].Endianness, EMConfig[Meter].DataType,
                                       EMConfig[Meter].PDivisor) < 0) {
                    var[x] = -var[x];
                }
            }
            break;

        case MM_ABB:
            for (x = 0; x < 3; x++) {
                if (receiveMeasurement(buf, x + 5u, EMConfig[Meter].Endianness, EMConfig[Meter].DataType,
                                       EMConfig[Meter].PDivisor) < 0) {
                    var[x] = -var[x];
                }
            }
            break;
    }

    // all OK
    return 1;
}

EVSEModbus evseModbus;