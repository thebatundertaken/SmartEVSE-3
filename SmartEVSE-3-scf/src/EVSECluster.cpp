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

    balancedCurrent[0] = current;
}

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

bool EVSECluster::isEnoughCurrentAvailable() {
    // Is there at least 6A(configurable MinCurrent) available for a EVSE?
    if (isLoadBalancerDisabled()) {
        return isEnoughCurrentAvailableForOneEVSE();
    }

    uint8_t numActiveEVSE = 0;
    int16_t clusterCurrent = 0;
    for (uint8_t n = 0; n < CLUSTER_NUM_EVSES; n++) {
        if (balancedState[n] == STATE_C_CHARGING) {
            numActiveEVSE++;
            clusterCurrent += balancedCurrent[n];
        }
    }

    if (numActiveEVSE == 0) {
        // No active (charging) EVSE's, it means an EVSE has detected a vehicle and
        // wants to start charging
        return isEnoughCurrentAvailableForOneEVSE();
    }

    if (evseController.mode == MODE_SOLAR) {
        // check if we can split the available current between all active EVSE's
        return clusterCurrent <= ((evseController.minEVCurrent * 10) * numActiveEVSE);
    }

    // When load balancing is active, and we are the Master, maxCircuit limits the
    // max total current
    int16_t measuredCurrent = getHouseMeasuredCurrent();
    if (isLoadBalancerMaster()) {
        if ((numActiveEVSE * evseController.minEVCurrent) > maxCircuit) {
            return false;
        }
    }

    int16_t baseload = _max(measuredCurrent - clusterCurrent, 0) + (numActiveEVSE * (evseController.minEVCurrent * 10));

    return baseload < (evseController.maxMains * 10);
}

bool EVSECluster::isEnoughCurrentAvailableForOneEVSE() {
    // TODO SCF fix global Isum
    if (evseController.mode == MODE_SOLAR) {
        return Isum < (evseController.solarStartCurrent * -10);
    }

    // Using _max due to solar surplus negative values
    int16_t measuredCurrent = _max(getHouseMeasuredCurrent(), 0);

    // There should be at least MIN_EV_CURRENT (default 6A) available
    if (measuredCurrent <= ((evseController.maxMains - evseController.minEVCurrent) * 10)) {
        return true;
    }

    sprintf(sprintfStr, "[EVSECluster] Not enough power for 1 EVSE. measuredCurrent=%d; maxMains=%d; minEVCurrent=%d",
            measuredCurrent, evseController.maxMains, evseController.minEVCurrent);
    EVSELogger::debug(sprintfStr);

    return false;
}

void EVSECluster::resetBalancedStates() {
    for (uint8_t n = 1; n < CLUSTER_NUM_EVSES; n++) {
        // Yes, disable old active Node states
        balancedState[n] = STATE_A_STANDBY;
        // reset ChargeCurrent to 0
        balancedCurrent[n] = 0;
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

    balancedCurrent[0] = current;
    evseController.setChargeCurrent(balancedCurrent[0]);
}

/**
 * Master checks node status requests, and responds with new state
 * Master -> Node
 *
 * @param uint8_t NodeAdr (1-7)
 */
void EVSECluster::processAllNodeStates(uint8_t nodeNr) {
    uint16_t values[2];
    bool write = false;

    values[0] = balancedState[nodeNr];

    bool current = isEnoughCurrentAvailable();
    // Yes enough current
    if (current) {
        if (balancedError[nodeNr] & (ERROR_FLAG_LESS_6A | ERROR_FLAG_NO_SUN)) {
            // Clear Error flags
            balancedError[nodeNr] &= ~(ERROR_FLAG_LESS_6A | ERROR_FLAG_NO_SUN);
            write = true;
        }
    }

    // Check EVSE for request to charge states
    switch (balancedState[nodeNr]) {
        case STATE_A_STANDBY:
            break;

            // Request to charge A->B
        case STATE_MODBUS_COMM_B:
            // check if we have enough current
            if (current) {
                // Mark Node EVSE as active (State B)
                balancedState[nodeNr] = STATE_B_VEHICLE_DETECTED;
                // Initially set current to lowest setting
                balancedCurrent[nodeNr] = evseController.minEVCurrent * 10;
                values[0] = STATE_MODBUS_COMM_B_OK;
                write = true;
            } else {
                // We do not have enough current to start charging
                // Make sure the Node does not start charging by setting current to 0
                balancedCurrent[nodeNr] = 0;
                // Error flags cleared?
                if ((balancedError[nodeNr] & (ERROR_FLAG_LESS_6A | ERROR_FLAG_NO_SUN)) == 0) {
                    if (evseController.mode == MODE_SOLAR) {
                        // Solar mode: No Solar Power available
                        balancedError[nodeNr] |= ERROR_FLAG_NO_SUN;
                    } else {
                        // Normal or Smart Mode: Not enough current available
                        balancedError[nodeNr] |= ERROR_FLAG_LESS_6A;
                    }
                    write = true;
                }
            }
            break;

            // request to charge B->C
        case STATE_MODBUS_COMM_C:
            balancedCurrent[nodeNr] = 0;
            // For correct baseload calculation set current to zero check if we have
            // enough current
            if (current) {
                // Mark Node EVSE as Charging (State C)
                balancedState[nodeNr] = STATE_C_CHARGING;
                // Calculate charge current for all connected EVSE's
                calcBalancedChargeCurrent();
                values[0] = STATE_MODBUS_COMM_C_OK;
                write = true;
            } else {
                // We do not have enough current to start charging
                // Error flags cleared?
                if ((balancedError[nodeNr] & (ERROR_FLAG_LESS_6A | ERROR_FLAG_NO_SUN)) == 0) {
                    if (evseController.mode == MODE_SOLAR) {
                        // Solar mode: No Solar Power available
                        balancedError[nodeNr] |= ERROR_FLAG_NO_SUN;
                    } else {
                        // Normal or Smart Mode: Not enough current available
                        balancedError[nodeNr] |= ERROR_FLAG_LESS_6A;
                    }
                    write = true;
                }
            }
            break;
    }

    if (write) {
        // Write State and Error to Node
        values[1] = balancedError[nodeNr];
        evseModbus.sendNewStatusToNode(nodeNr + 1, values);
    }
}

// Total power used by EVSEs cluster. In memory value, might not match real value even using evMeter
int16_t EVSECluster::getClusterCurrent() {
    int16_t clusterCurrent = 0;
    for (uint8_t n = 0; n < CLUSTER_NUM_EVSES; n++) {
        if (balancedState[n] == STATE_C_CHARGING || balancedState[n] == STATE_DISCONNECT_IN_PROGRESS) {
            clusterCurrent += balancedCurrent[n];
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

// Calculates balancedCurrent PWM current for each EVSE
void EVSECluster::calcBalancedChargeCurrent() {
    uint16_t mycableMaxCapacity = evseController.cableMaxCapacity;
    if (mycableMaxCapacity == 0) {
        // No cable info (maybe CONFIG_SOCKET and no cable attached)
        mycableMaxCapacity = evseController.maxDeviceCurrent;
    }

    // Max current (Amps *10) available for all EVSE's (can be negative)
    int16_t maxCurrentAvailable = _min(evseController.maxMains, mycableMaxCapacity) * 10;
    // Override current (received from Modbus master)
    if (amIWorkerNode() && overrideCurrent > 0) {
        maxCurrentAvailable = overrideCurrent;
    }

    sprintf(sprintfStr,
            "[EVSECluster] maxDeviceCurrent=%d; maxCurrentAvailable=%d; maxMains=%d; minEVCurrent=%d; cableMax=%d",
            evseController.maxDeviceCurrent, maxCurrentAvailable, evseController.maxMains, evseController.minEVCurrent,
            mycableMaxCapacity);
    EVSELogger::debug(sprintfStr);

    int16_t measuredCurrent, clusterCurrent, baseload, difference, numEVSEScharging, chargeCurrent, newChargeCurrent;
    switch (evseController.mode) {
        case MODE_SMART:
            // Store in variable to avoid race conditions
            chargeCurrent = evseController.getChargeCurrent();
            // Total power demand from the house (including charging EVSEs)
            // It can be negative due to solar panels surplus
            measuredCurrent = getHouseMeasuredCurrent();
            // Total power used by EVSEs cluster. In memory value, might not match real value even using evMeter
            clusterCurrent = getClusterCurrent();
            // Base load (house only, EXCLUDING charging EVSEs)
            // Using _max(X, 0) to avoid negative values due to solar panels surplus
            baseload = _max(measuredCurrent, 0) - clusterCurrent;

            if (baseload < 0) {
                // clusterCurrent mismatch because clusterCurrent is an in-memory value not the real power
                // consumption reading. It usually happens when a vehicle start charging (it takes 5-10
                // seconds for the vehicle to charge at full load)
                sprintf(sprintfStr,
                        "[EVSECluster] clusterCurrent mismatch! Vehicle has just started charging?? "
                        "baseload=%d; clusterCurrent=%d",
                        baseload, clusterCurrent);
                EVSELogger::info(sprintfStr);

                // Recalc baseload using minEVCurrent value as reference, because a charging vehicle will
                // consume a very minimum of minEVCurrent
                numEVSEScharging = _max(getNumEVSEScharging(), 1);
                baseload = _max(measuredCurrent, 0) -
                           _max(clusterCurrent, evseController.minEVCurrent * 10 * numEVSEScharging);
                // Using _max(X, 0) to avoid negative values due to solar panels surplus
                baseload = _max(baseload, 0);
            }

            // Using _max(X, 0) because overrideCurrent, new menu configuration, etc... might lead to negative values
            newChargeCurrent = _max(maxCurrentAvailable - baseload, 0);
            difference = newChargeCurrent - chargeCurrent;

            // difference > 0: Spare power, slowly increase current by 1/4th of difference
            // difference < 0: Exceeded power, immediately decrease current to match maximum available
            // diference == 0: Perfectly balanced
            if (difference > 0) {
                // Fix: use ceil to avoid small difference value increases 0 result (ex: diff=1 increase (1/4 => 0))
                newChargeCurrent = chargeCurrent + ceil((float)difference / REBALANCE_SLOW_INCREASE_FACTOR);
            }

            sprintf(sprintfStr,
                    "[EVSECluster] newChargeCurrent=%d; chargeCurrent=%d; maxCurrentAvailable=%d; measuredCurrent=%d; "
                    "clusterCurrent=%d; baseload=%d; difference=%d",
                    newChargeCurrent, evseController.getChargeCurrent(), maxCurrentAvailable, measuredCurrent,
                    clusterCurrent, baseload, difference);
            memcpy(DEBUG_LAST_CHARGE_REBALANCE_CALC, sprintfStr, sizeof(sprintfStr));
            EVSELogger::debug(sprintfStr);

            break;

        case MODE_SOLAR:
            newChargeCurrent = availableCurrentWithSolar();
            break;

        case MODE_NORMAL:
            newChargeCurrent = isLoadBalancerMaster() ? (maxCircuit * 10) : maxCurrentAvailable;
            break;
    }

    if (isLoadBalancerDisabled()) {
        sprintf(sprintfStr, "[EVSECluster] Standalone EVSE value=%d", newChargeCurrent);
        EVSELogger::debug(sprintfStr);

        resetBalancedStates();
        balancedMax[0] = evseController.getChargeCurrent();
        balancedCurrent[0] = newChargeCurrent;
        return;
    }

    sprintf(sprintfStr, "[EVSECluster] Cluster (LoadBl=%d) EVSE value=%d\n", LoadBl, newChargeCurrent);
    EVSELogger::info(sprintfStr);
    calcBalancedChargeCurrentCluster(newChargeCurrent);
}

void EVSECluster::calcBalancedChargeCurrentCluster(int16_t adjustedCurrent) {
    // Do not exceed maxCircuit
    if ((adjustedCurrent > (getMaxCircuit() * 10))) {
        adjustedCurrent = getMaxCircuit() * 10;
    }

    if (isLoadBalancerMaster()) {
        balancedMax[0] = evseController.getChargeCurrent();
    }

    int16_t activeMax = 0, totalCurrent = 0, numEVSEScharging = 0;
    for (uint8_t i = 0; i < CLUSTER_NUM_EVSES; i++) {
        if (balancedState[i] == STATE_C_CHARGING) {
            // Count nr of Active (Charging) EVSE's
            numEVSEScharging++;
            // Calculate total Max Amps for all active EVSEs
            activeMax += balancedMax[i];
            // Calculate total of all set charge currents
            totalCurrent += balancedCurrent[i];
        }
    }

    if ((adjustedCurrent < (numEVSEScharging * (evseController.minEVCurrent * 10))) ||
        (evseController.mode == MODE_SOLAR && Isum > 10 &&
         (getHouseMeasuredCurrent() > (evseController.maxMains * 10)))) {
        EVSELogger::info("[EVSECluster] Not enough power for EVSE cluster");
        evseController.errorFlags |= ERROR_FLAG_LESS_6A;
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
        if ((balancedState[n] == STATE_C_CHARGING) && (!currentSet[n]) && (averageCurrent >= balancedMax[n])) {
            // Set current to Maximum allowed for this EVSE
            balancedCurrent[n] = balancedMax[n];
            // mark this EVSE as set.
            currentSet[n] = 1;
            // decrease counter of active EVSE's
            numEVSEScharging--;
            // Update total current to new (lower) value
            clusterCurrent -= balancedCurrent[n];
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
            if ((balancedState[n] == STATE_C_CHARGING) && (!currentSet[n])) {
                // Set current to Average
                balancedCurrent[n] = clusterCurrent / numEVSEScharging;
                // mark this EVSE as set.
                currentSet[n] = 1;
                // decrease counter of active EVSE's
                numEVSEScharging--;
                // Update total current to new (lower) value
                clusterCurrent -= balancedCurrent[n];
            }
        } while (++n < CLUSTER_NUM_EVSES && numEVSEScharging);
    }
}

int16_t EVSECluster::availableCurrentWithSolar() {
    // Allow Import of power from the grid when solar charging
    int16_t sumImport = Isum - (10 * evseController.solarImportCurrent);
    int16_t balancedCurrent = 0;

    if (sumImport < 0) {
        // negative, we have surplus (solar) power available
        // more then 1A available, increase balancedCurrent charge current with
        // 0.5A
        if (sumImport < -10) {
            balancedCurrent = balancedCurrent + 5;
        } else {
            // less then 1A available, increase with 0.1A
            balancedCurrent = balancedCurrent + 1;
        }
    } else {
        // positive, we use more power than generated
        if (sumImport > 20) {
            // we use atleast 2A more then available, decrease balancedCurrent
            // charge current.
            balancedCurrent = balancedCurrent - (sumImport / 2);
        } else if (sumImport > 10) {
            // we use 1A more then available, decrease with 0.5A
            balancedCurrent = balancedCurrent - 5;
        } else if (sumImport > 3) {
            // we still use > 0.3A more then available, decrease with 0.1A
            // if we use <= 0.3A we do nothing
            balancedCurrent = balancedCurrent - 1;
        }
    }

    int numEVSEScharging = 0;
    for (uint8_t n = 0; n < CLUSTER_NUM_EVSES; n++) {
        if (balancedState[n] == STATE_C_CHARGING) {
            numEVSEScharging++;
        }
    }

    bool lowCurrent = false;

    // If balancedCurrent is below MinCurrent or negative, make sure it's set to
    // MinCurrent.
    if ((balancedCurrent < (numEVSEScharging * evseController.minEVCurrent * 10)) || (balancedCurrent < 0)) {
        lowCurrent = true;
        balancedCurrent = numEVSEScharging * evseController.minEVCurrent * 10;
    }

    if (lowCurrent) {
        // Check to see if we have to continue charging on solar power alone
        if ((numEVSEScharging > 0) && (sumImport > 10)) {
            if (evseController.solarStopTimer == 0 && evseController.solarStopTimeMinutes != 0) {
                // Convert minutes into seconds
                evseController.setSolarStopTimer(evseController.solarStopTimeMinutes * 60);
            }
        }
    } else {
        if (evseController.solarStopTimer != 0) {
            evseController.setSolarStopTimer(0);
        }
    }

    return balancedCurrent;
}

int16_t EVSECluster::getHouseMeasuredCurrent() {
    // SCF fix: imeasured holds sum all Irms of all channels instead of
    // imeasured holds highest Irms of all channels
    // SCF fix: imeasured holds sum all Irms of all channels instead of max
    // Max of all Phases (Amps *10, 23 = 2.3A) of mains power
    int16_t imeasured = 0;

    for (uint8_t x = 0; x < 3; x++) {
        /*if (evseController.Irms[x] > imeasured)
            imeasured = evseController.Irms[x];*/
        imeasured += evseController.Irms[x];
    }
    return imeasured;
}

void EVSECluster::onCTDataReceived() {
    adjustChargeCurrent();
}

void EVSECluster::adjustChargeCurrent() {
    if (!(evseController.errorFlags & ERROR_FLAG_CT_NOCOMM)) {
        // Calculate dynamic charge current for connected EVSE's
        calcBalancedChargeCurrent();
    }

    if (isLoadBalancerMaster()) {
        // If there is no enough power
        if (evseController.errorFlags & ERROR_FLAG_LESS_6A) {
            // Set all EVSE's to State A standby
            resetBalancedStates();
            evseModbus.broadcastErrorFlagsToWorkerNodes(evseController.errorFlags);
        } else {
            evseModbus.broadcastMasterBalancedCurrent(balancedCurrent);
        }
    }

    evseController.setChargeCurrent(balancedCurrent[0]);
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