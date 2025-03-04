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

#include "EVSECluster.h"

#include <Arduino.h>
#include <Preferences.h>

#include "EVSEController.h"
#include "EVSELogger.h"
#include "EVSEModbus.h"

const char* PREFS_CLUSTER_NAMESPACE = "settings";
const char* PREFS_LOADBL_KEY = "LoadBl";
const char* PREFS_MAXCIRCUIT_KEY = "MaxCircuit";

void EVSECluster::setMasterNodeBalancedState(uint8_t state) {
    if (!evseCluster.amIMasterOrLBDisabled()) {
        return;
    }

    balancedState[0] = state;
}

void EVSECluster::setMasterNodeBalancedMax(uint16_t newBalancedMax) {
    if (!evseCluster.amIMasterOrLBDisabled()) {
        return;
    }

    balancedMax[0] = newBalancedMax;
}

void EVSECluster::setMasterNodeBalancedCurrent(uint16_t current) {
    if (!evseCluster.amIMasterOrLBDisabled()) {
        return;
    }

    balancedChargeCurrent[0] = current;
}

#if EVSE_FEATFLAG_ENABLE_POWERSHARE
void EVSECluster::setMasterNodeControllerMode(uint8_t mode) {
    if (!evseCluster.amIMasterOrLBDisabled()) {
        return;
    }

    evseModbus.broadcastControllerModeToNodes(mode);
}

void EVSECluster::setMasterNodeErrorflags(uint8_t errorFlags) {
    if (!evseCluster.amIMasterOrLBDisabled()) {
        return;
    }

    evseModbus.broadcastErrorFlagsToWorkerNodes(errorFlags);
}

void EVSECluster::setMasterSolarStopTimer(uint16_t solarStopTimer) {
    if (!evseCluster.amIMasterOrLBDisabled()) {
        return;
    }

    evseModbus.broadcastSolarStopTimerToNodes(solarStopTimer);
}
#endif

bool EVSECluster::amIMasterOrLBDisabled() {
    return LoadBl == LOAD_BALANCER_MASTER || LoadBl == LOAD_BALANCER_DISABLED;
}

// Cluster mode: node 1 = master, workers 2-8
bool EVSECluster::amIWorkerNode() {
    return LoadBl >= 2;
}

bool EVSECluster::isLoadBalancerMaster() {
    return LoadBl == LOAD_BALANCER_MASTER;
}

bool EVSECluster::isLoadBalancerDisabled() {
    return LoadBl == LOAD_BALANCER_DISABLED;
}

bool EVSECluster::isEnoughCurrentAvailableForAnotherEVSE() {
    // Is there at least 6A(configurable MinCurrent) available for a EVSE?
    if (isLoadBalancerDisabled()) {
        return isEnoughCurrentAvailableForOneEVSE();
    }

    uint8_t numActiveEVSE = getNumEVSEScharging();
    if (numActiveEVSE == 0) {
        // No active (charging) EVSE's, it means an EVSE has detected a vehicle and wants to start charging
        return isEnoughCurrentAvailableForOneEVSE();
    }

    int16_t clusterCurrent = getClusterCurrent();
    if (evseController.mode == MODE_SOLAR) {
        // check if we can split the available current between all active EVSE's
        return clusterCurrent <= ((evseController.minEVCurrent * 10) * numActiveEVSE);
    }

    // When load balancing is active, and we are the Master, maxCircuit limits the max total current
    if (isLoadBalancerMaster()) {
        if ((numActiveEVSE * evseController.minEVCurrent) > maxCircuit) {
            return false;
        }
    }

    const int16_t measuredCurrent = evseController.getMainsMeasuredCurrent(false);
    int16_t futureLoad =
        _max(measuredCurrent - clusterCurrent, 0) + (numActiveEVSE * (evseController.minEVCurrent * 10));

    return futureLoad <= evseController.getMaxCurrentAvailable();
}

bool EVSECluster::isEnoughCurrentAvailableForOneEVSE() {
    if (evseController.mode == MODE_SOLAR) {
        // Solar surplus (negative value) must exceed 4A (configurable), otherwise ERROR_FLAG_NO_SUN
        return evseController.getMainsMeasuredCurrent(true) < (evseController.solarStartCurrent * -10);
    }

    const int16_t measuredCurrent = evseController.getMainsMeasuredCurrent(false);

    // There should be at least MIN_EV_CURRENT (default 5A) available
    if (measuredCurrent <= (evseController.getMaxCurrentAvailable() - (evseController.minEVCurrent * 10))) {
        return true;
    }

    sprintf(sprintfStr, "[EVSECluster] Not enough power for 1 EVSE. measuredCurrent=%d; maxMains=%d; minEVCurrent=%d",
            evseController.getMainsMeasuredCurrent(true), evseController.maxMains, evseController.minEVCurrent);
    EVSELogger::debug(sprintfStr);

    return false;
}

void EVSECluster::resetBalancedStates() {
    for (uint8_t n = 1; n < CLUSTER_NUM_EVSES; n++) {
        // Yes, disable old active Node states
        balancedState[n] = STATE_A_STANDBY;
        // reset ChargeCurrent to 0
        balancedChargeCurrent[n] = 0;
    }
}

void EVSECluster::setClusterNodeStatus(uint8_t nodeNr, uint8_t state, uint16_t error, uint16_t maxChargeCurrent) {
    // Node State
    balancedState[nodeNr] = state;
    // Node Error status
    balancedError[nodeNr] = error;
    // Node Max ChargeCurrent (0.1A)
    balancedMax[nodeNr] = maxChargeCurrent;
}

void EVSECluster::setWorkerNodeMasterBalancedCurrent(uint16_t current) {
    // Modbus broadcasting, master node will receive its own broadcasted messages
    if (!amIWorkerNode()) {
        return;
    }

    balancedChargeCurrent[0] = current;
    evseController.setChargeCurrent(balancedChargeCurrent[0]);
}

#if EVSE_FEATFLAG_ENABLE_POWERSHARE
/**
 * Master checks node status requests, and responds with new state
 * @param uint8_t NodeAdr (1-7)
 */
void EVSECluster::processAllNodeStates(uint8_t nodeNr) {
    uint16_t values[2];
    bool propagateChangesToNodes = false;

    values[0] = balancedState[nodeNr];

    if (isEnoughCurrentAvailableForAnotherEVSE()) {
        if (balancedError[nodeNr] & (ERROR_FLAG_LESS_6A | ERROR_FLAG_NO_SUN)) {
            balancedError[nodeNr] &= ~(ERROR_FLAG_LESS_6A | ERROR_FLAG_NO_SUN);
            propagateChangesToNodes = true;
        }

        switch (balancedState[nodeNr]) {
            case STATE_MODBUS_COMM_B:
                // Request to charge A->B
                // Mark Node EVSE as active (State B)
                balancedState[nodeNr] = STATE_B_VEHICLE_DETECTED;
                // Initially set current to lowest setting
                balancedChargeCurrent[nodeNr] = evseController.minEVCurrent * 10;
                values[0] = STATE_MODBUS_COMM_B_OK;
                propagateChangesToNodes = true;
                break;

            case STATE_MODBUS_COMM_C:
                // request to charge B->C. Mark Node EVSE as Charging (State C)
                balancedState[nodeNr] = STATE_C_CHARGING;
                // For correct baseload calculation set current to zero check if we have enough current
                balancedChargeCurrent[nodeNr] = 0;
                recalcBalancedChargeCurrent();
                values[0] = STATE_MODBUS_COMM_C_OK;
                propagateChangesToNodes = true;
                break;
        }
    } else {
        // We do not have enough current to start charging
        // Make sure the Node does not start charging by setting current to 0
        balancedChargeCurrent[nodeNr] = 0;
        if ((balancedError[nodeNr] & (ERROR_FLAG_LESS_6A | ERROR_FLAG_NO_SUN)) == 0) {
            balancedError[nodeNr] |= (evseController.mode == MODE_SOLAR) ? ERROR_FLAG_NO_SUN : ERROR_FLAG_LESS_6A;
            propagateChangesToNodes = true;
        }
    }

    if (propagateChangesToNodes) {
        // Write State and Error to Node
        values[1] = balancedError[nodeNr];
        evseModbus.sendNewStatusToNode(nodeNr + 1, values);
    }
}
#endif

// Total power used by EVSEs cluster. In memory value, might not match real value even using evMeter
int16_t EVSECluster::getClusterCurrent() {
    int16_t clusterCurrent = 0;
    for (uint8_t n = 0; n < CLUSTER_NUM_EVSES; n++) {
        if (balancedState[n] == STATE_C_CHARGING || balancedState[n] == STATE_DISCONNECT_IN_PROGRESS) {
            clusterCurrent += balancedChargeCurrent[n];
        }
    }

    return clusterCurrent;
}

int8_t EVSECluster::getNumEVSEScharging() {
    int8_t numEVSEScharging = 0;
    for (uint8_t n = 0; n < CLUSTER_NUM_EVSES; n++) {
        if (balancedState[n] == STATE_C_CHARGING || balancedState[n] == STATE_DISCONNECT_IN_PROGRESS) {
            numEVSEScharging++;
        }
    }

    return numEVSEScharging;
}

// Calculates balancedChargeCurrent PWM current for each EVSE
void EVSECluster::recalcBalancedChargeCurrent() {
    // Max current (Amps *10) available for all EVSE's (can be negative)
    int16_t maxCurrentAvailable = evseController.getMaxCurrentAvailable();
    // Override current (received from Modbus master)
    if (amIWorkerNode() && overrideCurrent > 0) {
        maxCurrentAvailable = overrideCurrent;
    }

    sprintf(sprintfStr, "[EVSECluster] maxCurrentAvailable=%d; minEVCurrent=%d", maxCurrentAvailable,
            evseController.minEVCurrent);
    EVSELogger::debug(sprintfStr);

    int16_t newChargeCurrent = 0;
    switch (evseController.mode) {
        case MODE_SMART:
            newChargeCurrent = (evseController.state == STATE_A_STANDBY)
                                   ? maxCurrentAvailable
                                   : calcNewChargeCurrentWithSmart(maxCurrentAvailable);
            break;

        case MODE_SOLAR:
            newChargeCurrent = calcNewChargeCurrentWithSolar(maxCurrentAvailable);
            break;

        default:  // case MODE_NORMAL:
            newChargeCurrent = isLoadBalancerMaster() ? (maxCircuit * 10) : maxCurrentAvailable;
            break;
    }

    if (isLoadBalancerDisabled()) {
        sprintf(sprintfStr, "[EVSECluster] recalcBalancedChargeCurrent::StandaloneEVSE newChargeCurrent=%d",
                newChargeCurrent);
        EVSELogger::debug(sprintfStr);

        resetBalancedStates();
        balancedMax[0] = evseController.getChargeCurrent();
        balancedChargeCurrent[0] = newChargeCurrent;
    } else {
        recalcBalancedChargeCurrentCluster(newChargeCurrent);
    }
}

// TODO not tested cluster of EVSEs use at your own risk
void EVSECluster::recalcBalancedChargeCurrentCluster(int16_t newChargeCurrent) {
    // Do not exceed maxCircuit
    int16_t adjustedCurrent = _min(getMaxCircuit() * 10, newChargeCurrent);

    if (isLoadBalancerMaster()) {
        balancedMax[0] = evseController.getChargeCurrent();
    }

    int16_t activeMax = 0, totalCurrent = 0, numEVSEScharging = 0;
    for (uint8_t i = 0; i < CLUSTER_NUM_EVSES; i++) {
        if (balancedState[i] == STATE_C_CHARGING || balancedState[i] == STATE_DISCONNECT_IN_PROGRESS) {
            // Count nr of Active (Charging) EVSE's
            numEVSEScharging++;
            // Calculate total Max Amps for all active EVSEs
            activeMax += balancedMax[i];
            // Calculate total of all set charge currents
            totalCurrent += balancedChargeCurrent[i];
        }
    }

    const int16_t measuredCurrent = evseController.getMainsMeasuredCurrent(false);
    if ((adjustedCurrent < (numEVSEScharging * (evseController.minEVCurrent * 10))) ||
        (evseController.mode == MODE_SOLAR && evseController.getMainsMeasuredCurrent(true) > 10 &&
         (measuredCurrent > evseController.getMaxCurrentAvailable()))) {
        EVSELogger::info("[EVSECluster] Not enough power for EVSE cluster");
        evseController.onNotEnoughPower();
        return;
    }

    // limit to total maximum Amps (of all active EVSE's)
    int16_t clusterCurrent = _min(adjustedCurrent, activeMax);

    // Calculate average current per EVSE
    char currentSet[CLUSTER_NUM_EVSES] = {0, 0, 0, 0, 0, 0, 0, 0};
    int averageCurrent;
    uint8_t n = 0;
    do {
        // Average current for all active EVSE's
        averageCurrent = clusterCurrent / numEVSEScharging;

        // Check for EVSE's that have a lower MAX current
        // Active EVSE, and current not yet calculated?
        if ((balancedState[n] == STATE_C_CHARGING || balancedState[n] == STATE_DISCONNECT_IN_PROGRESS) &&
            (!currentSet[n]) && (averageCurrent >= balancedMax[n])) {
            // Set current to Maximum allowed for this EVSE
            balancedChargeCurrent[n] = balancedMax[n];
            // mark this EVSE as set.
            currentSet[n] = 1;
            // decrease counter of active EVSE's
            numEVSEScharging--;
            // Update total current to new (lower) value
            clusterCurrent -= balancedChargeCurrent[n];
            // check all EVSE's again
            n = 0;
        } else {
            n++;
        }
    } while (n < CLUSTER_NUM_EVSES && numEVSEScharging);

    // All EVSE's which had a Max current lower then the average are set.
    // Now calculate the current for the EVSE's which had a higher Max current
    // Any Active EVSE's left?
    if (numEVSEScharging > 0) {
        n = 0;
        do {
            // Check for EVSE's that are not set yet
            // Active EVSE, and current not yet calculated?
            if ((balancedState[n] == STATE_C_CHARGING || balancedState[n] == STATE_DISCONNECT_IN_PROGRESS) &&
                (!currentSet[n])) {
                // Set current to Average
                balancedChargeCurrent[n] = clusterCurrent / numEVSEScharging;
                // mark this EVSE as set.
                currentSet[n] = 1;
                // decrease counter of active EVSE's
                numEVSEScharging--;
                // Update total current to new (lower) value
                clusterCurrent -= balancedChargeCurrent[n];
            }
        } while (++n < CLUSTER_NUM_EVSES && numEVSEScharging);
    }
}

int16_t EVSECluster::calcNewChargeCurrentWithSmart(int16_t maxCurrentAvailable) {
    // Total power demand from the house (including charging EVSEs)
    int16_t measuredCurrent = evseController.getMainsMeasuredCurrent(false);
    // Total power used by EVSEs cluster. In memory value, might not match real value even using evMeter
    int16_t clusterCurrent = getClusterCurrent();
    // Base load (house only, EXCLUDING charging EVSEs)
    // Using _max(X, 0) to avoid negative values due to solar panels surplus
    int16_t baseload = measuredCurrent - clusterCurrent;

    if (baseload < 0) {
        // clusterCurrent mismatch because clusterCurrent is an in-memory value not the real power
        // consumption reading. It usually happens when a vehicle start charging (it takes 5-10
        // seconds for the vehicle to charge at full load)
        sprintf(sprintfStr,
                "[EVSECluster] clusterCurrent mismatch! Vehicle has just started charging?? "
                "baseload=%d; clusterCurrent=%d",
                baseload, clusterCurrent);
        EVSELogger::debug(sprintfStr);

        // Recalc baseload using minEVCurrent value as reference, because a charging vehicle will
        // consume a very minimum of minEVCurrent
        baseload =
            measuredCurrent - _max(clusterCurrent, evseController.minEVCurrent * 10 * _max(getNumEVSEScharging(), 1));
        // Using _max(X, 0) to avoid negative values due to solar panels surplus
        baseload = _max(baseload, 0);
    }

    // Solar boost
    int16_t solarBoostCurrent = evseController.calcSolarBoostCurrent();
    if (solarBoostCurrent > 0) {
        maxCurrentAvailable += solarBoostCurrent;
    }
    sprintf(sprintfStr, "[EVSECluster] [SolarBoost] solarBoostCurrent=%d; maxCurrentBoostedAvailable=%d",
            solarBoostCurrent, maxCurrentAvailable);
    EVSELogger::debug(sprintfStr);

    // Using _max(X, 0) because overrideCurrent, new menu configuration, etc... might lead to negative values
    int16_t newChargeCurrent = _max(maxCurrentAvailable - baseload, 0);
    // Store in variable to avoid race conditions
    int16_t chargeCurrent = evseController.getChargeCurrent();
    int16_t difference = newChargeCurrent - chargeCurrent;

    // difference > 0: Spare power, slowly increase current by 1/4th of difference
    // difference < 0: Exceeded power, immediately decrease current to match maximum available
    // diference == 0: Perfectly balanced
    if (difference > 0) {
        // Fix: use ceil to avoid small difference value increases 0 result (ex: diff=1 increase (1/4 => 0))
        newChargeCurrent = chargeCurrent + ceil((float)difference / REBALANCE_SLOW_INCREASE_FACTOR);
    }

    sprintf(sprintfStr,
            "[EVSECluster] SMART newChargeCurrent=%d; chargeCurrent=%d; maxCurrentAvailable=%d; measuredCurrent=%d; "
            "clusterCurrent=%d; baseload=%d; difference=%d",
            newChargeCurrent, evseController.getChargeCurrent(), maxCurrentAvailable,
            evseController.getMainsMeasuredCurrent(true), clusterCurrent, baseload, difference);
    memcpy(DEBUG_LAST_CHARGE_REBALANCE_CALC, sprintfStr, sizeof(sprintfStr));
    EVSELogger::debug(sprintfStr);

    return newChargeCurrent;
}

int16_t EVSECluster::calcNewChargeCurrentWithSolar(int16_t maxCurrentAvailable) {
    // Allow Import of power from the grid when solar charging
    int16_t measuredCurrent = evseController.getMainsMeasuredCurrent(true);
    int16_t diffSurplusImport = measuredCurrent - (evseController.solarImportCurrent * 10);
    int16_t increment = 0;

    if (diffSurplusImport < 0) {
        // negative, we have solar surplus power available
        // more then 1A available, increase balancedChargeCurrent charge current with 0.5A, otherwise increase with 0.1A
        increment += (diffSurplusImport < -10) ? 5 : 1;
    } else {
        // positive (importing power from grid), we use more power than generated
        if (diffSurplusImport > 20) {
            // we use at least 2A more then available, decrease 50%
            increment -= (diffSurplusImport / 2);
        } else if (diffSurplusImport > 10) {
            // we use 1A more then available, decrease with 0.5A
            increment -= 5;
        } else if (diffSurplusImport > 3) {
            // we still use > 0.3A more then available, decrease with 0.1A
            increment -= 1;
        } else {
            // if we use <= 0.3A we do nothing
        }
    }

    int16_t newChargeCurrent = evseController.getChargeCurrent() + increment;
    bool lowCurrent = (increment < 0) || (newChargeCurrent < (evseController.minEVCurrent * 10));

    // Make sure newChargeCurrent is at least MIN_EV and do not exceed maxCurrentAvailable
    newChargeCurrent = _max(newChargeCurrent, (evseController.minEVCurrent * 10));
    newChargeCurrent = _min(newChargeCurrent, maxCurrentAvailable);

    if (lowCurrent) {
        // Check to see if we have to continue charging on solar power or stop (importing more than 1A)
        if (diffSurplusImport > 10) {
            evseController.onSolarLowPower();
        }
    } else {
        evseController.onSolarChargingEnoughPower();
    }

    sprintf(sprintfStr,
            "[EVSECluster] SOLAR newChargeCurrent=%d; chargeCurrent=%d; maxCurrentAvailable=%d; measuredCurrent=%d; "
            "diffSurplusImport=%d; increment=%d",
            newChargeCurrent, evseController.getChargeCurrent(), maxCurrentAvailable, measuredCurrent,
            diffSurplusImport, increment);
    memcpy(DEBUG_LAST_CHARGE_REBALANCE_CALC, sprintfStr, sizeof(sprintfStr));
    EVSELogger::debug(sprintfStr);

    return newChargeCurrent;
}

void EVSECluster::onCTDataReceived() {
    adjustChargeCurrent();
}

void EVSECluster::adjustChargeCurrent() {
    if (!(evseController.errorFlags & ERROR_FLAG_CT_NOCOMM)) {
        // Calculate dynamic charge current for connected EVSE's
        recalcBalancedChargeCurrent();
    }

#if EVSE_FEATFLAG_ENABLE_POWERSHARE
    if (isLoadBalancerMaster()) {
        // If there is no enough power
        if (evseController.errorFlags & ERROR_FLAG_LESS_6A) {
            // Set all EVSE's to State A standby
            resetBalancedStates();
            evseModbus.broadcastErrorFlagsToWorkerNodes(evseController.errorFlags);
        } else {
            evseModbus.broadcastMasterBalancedCurrent(balancedChargeCurrent);
        }
    }
#endif

    evseController.setChargeCurrent(balancedChargeCurrent[0]);
}

void EVSECluster::readEpromSettings() {
    Preferences preferences;
    if (preferences.begin(PREFS_CLUSTER_NAMESPACE, true) != true) {
        EVSELogger::error("[EVSECluster] Unable to open preferences for EVSECluster");
        return;
    }

    bool firstRun = true;
    if (preferences.isKey(PREFS_LOADBL_KEY)) {
        firstRun = false;

        LoadBl = preferences.getUChar(PREFS_LOADBL_KEY, LOAD_BALANCER_DISABLED);
        maxCircuit = preferences.getUShort(PREFS_MAXCIRCUIT_KEY, MAX_CIRCUIT);
    }
    preferences.end();

    if (firstRun) {
        LoadBl = LOAD_BALANCER_DISABLED;
        maxCircuit = MAX_CIRCUIT;
        writeEpromSettings();
    }
}

void EVSECluster::validateSettings() {}

void EVSECluster::writeEpromSettings() {
    validateSettings();

    Preferences preferences;
    if (preferences.begin(PREFS_CLUSTER_NAMESPACE, false) != true) {
        EVSELogger::error("[EVSECluster] Unable to write preferences for EVSECluster");
        return;
    }

    preferences.putUChar(PREFS_LOADBL_KEY, LoadBl);
    preferences.putUShort(PREFS_MAXCIRCUIT_KEY, maxCircuit);

    preferences.end();
}

void EVSECluster::updateSettings() {
    writeEpromSettings();
}

void EVSECluster::resetSettings() {
    readEpromSettings();
}

void EVSECluster::setup() {
    readEpromSettings();
}

EVSECluster evseCluster;