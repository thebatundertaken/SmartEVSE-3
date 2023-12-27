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

#ifndef __EVSE_CLUSTER
#define __EVSE_CLUSTER

#include <Arduino.h>

#define CLUSTER_NUM_EVSES 8
#define LOAD_BALANCER_DISABLED 0
#define LOAD_BALANCER_MASTER 1

#define REBALANCE_SLOW_INCREASE_FACTOR 4

// Max current of the EVSE circuit breaker (A)
#define MAX_CIRCUIT 16

class EVSECluster {
   public:
    EVSECluster(){};

    uint8_t getLoadBl() { return LoadBl; };
    void setLoadBl(uint8_t value) { LoadBl = value; };

    uint16_t getMaxCircuit() { return maxCircuit; };
    void setMaxCircuit(uint16_t value) { maxCircuit = value; };

    bool isLoadBalancerMaster();
    bool isLoadBalancerDisabled();
    bool amIMasterOrLBDisabled();
    bool amIWorkerNode();

    void processAllNodeStates(uint8_t nodeNr);

    void setMasterNodeBalancedState(uint8_t state);
    void setMasterNodeBalancedMax(uint16_t newBalancedMax);
    void setMasterNodeBalancedCurrent(uint16_t current);
    void setMasterNodeControllerMode(uint8_t mode);
    void setMasterNodeErrorflags(uint8_t errorFlags);
    void setMasterSolarStopTimer(uint16_t solarStopTimer);

    void resetBalancedStates();
    void setClusterNodeStatus(uint8_t nodeNr, uint8_t state, uint16_t error, uint16_t maxChargeCurrent);
    void setWorkerNodeMasterBalancedCurrent(uint16_t current);

    void onCTDataReceived();

    bool isEnoughCurrentAvailableForAnotherEVSE();
    void adjustChargeCurrent();

    uint16_t getOverrideCurrent() { return overrideCurrent; };
    void setOverrideCurrent(uint16_t val) { overrideCurrent = val; }

    void updateSettings();
    void resetSettings();
    void setup();

    int16_t getClusterCurrent();
    char DEBUG_LAST_CHARGE_REBALANCE_CALC[255];

   private:
    // Load Balance Setting (0:Disable / 1:Master / 2-8:Node)
    uint8_t LoadBl = LOAD_BALANCER_DISABLED;
    // Max current of the EVSE circuit (A)
    uint16_t maxCircuit = MAX_CIRCUIT;
    // Temporary assigned current (Amps *10) (modbus)
    uint16_t overrideCurrent = 0;

    // Error state of EVSE
    uint16_t balancedError[CLUSTER_NUM_EVSES] = {0, 0, 0, 0, 0, 0, 0, 0};
    // State of all EVSE's 0=not active (state A), 1=charge request (State B),
    // 2= Charging (State C)
    uint8_t balancedState[CLUSTER_NUM_EVSES] = {0, 0, 0, 0, 0, 0, 0, 0};
    // Max Amps value per EVSE
    uint16_t balancedMax[CLUSTER_NUM_EVSES] = {0, 0, 0, 0, 0, 0, 0, 0};
    // Load Balance variables, Amps value per EVSE
    uint16_t balancedChargeCurrent[CLUSTER_NUM_EVSES] = {0, 0, 0, 0, 0, 0, 0, 0};
    char sprintfStr[255];

    bool isEnoughCurrentAvailableForOneEVSE();
    int16_t calcNewChargeCurrentWithSolar(int16_t maxCurrentAvailable);
    int16_t calcNewChargeCurrentWithSmart(int16_t maxCurrentAvailable);

    void recalcBalancedChargeCurrent();
    void recalcBalancedChargeCurrentCluster(int16_t adjustedCurrent);
    int8_t getNumEVSEScharging();

    void readEpromSettings();
    void writeEpromSettings();
    void validateSettings();
};

extern EVSECluster evseCluster;

#endif