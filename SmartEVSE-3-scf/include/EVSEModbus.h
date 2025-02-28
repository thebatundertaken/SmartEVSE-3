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

#ifndef __EVSE_MODBUS
#define __EVSE_MODBUS

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>

#include "EVSECluster.h"
#include "EVSEPin.h"
#include "ModbusClientRTU.h"
#include "ModbusServerRTU.h"

#define BROADCAST_ADR 0x09

// Type of EV electric meter (0: Disabled / Constants EM_*)
#define EV_METER_DISABLED 0
#define EV_METER_ADDRESS 12
#define MIN_EV_METER_ADDRESS 10
#define MAX_EV_METER_ADDRESS 247

// Type of PV electric meter (0: Disabled / Constants EM_*)
#define PV_METER_DISABLED 0
#define PV_METER_ADDRESS 11

// Mains Meter, 1= Sensorbox, 2=Phoenix, 3= Finder, 4= Eastron, 8=Custom
#define MAINS_METER_DISABLED 0
#define MM_SENSORBOX 1
#define MM_PHOENIX_CONTACT 2
#define MM_FINDER 3
#define MM_EASTRON 4
#define MM_ABB 5
#define MM_SOLAREDGE 6
#define MM_WAGO 7
#define MM_CUSTOM 8

#define MAINS_METER_ADDRESS 10
// What does Mains electric meter measure (0: Mains (Home+EVSE+PV) / 1:
// Home+EVSE / 2: Home)
#define MAINS_METER_MEASURE 0

// Type of Grid connected to Sensorbox (3 phases 4 Wire or 3 Wire, single phase)
#define GRID_4WIRE 0
#define GRID_3WIRE 1
#define GRID_SINGLE_PHASE 2

// Sensorbox v2 has always address 0x0A
#define MM_SENSORBOX_ADDRESS 0x0A

#define MODBUS_FUNCTION_READ_HOLDING_REGISTER 0x03
#define MODBUS_FUNCTION_READ_INPUT_REGISTER 0x04
#define MODBUS_FUNCTION_WRITE_SINGLE_REGISTER 0x06
#define MODBUS_FUNCTION_WRITE_MULTIPLE_REGISTER 0x10
#define MODBUS_FUNCTION_WRITE_ERROR_CODE_REGISTER 0x0001
#define MODBUS_FUNCTION_NODE_CONFIG_REGISTER 0x0108
#define MODBUS_FUNCTION_NODE_STATUS_REGISTER 0x0000
#define MODBUS_FUNCTION_SENSORBOX_GRID_REGISTER 0x800

// communication timeout (millis)
#define DEFAULT_CT_COMM_TIMEOUT 10000
// communication request periodicity (millis)
#define DEFAULT_CT_REQUEST_PERIOD 2000

#define MODBUS_TIMEOUT 4
// 1000ms timeout
#define ACK_TIMEOUT 1000

#define MODBUS_INVALID 0
#define MODBUS_OK 1
#define MODBUS_REQUEST 2
#define MODBUS_RESPONSE 3
#define MODBUS_EXCEPTION 4

#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS 0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE 0x03

#define MODBUS_EVSE_STATUS_START 0x0000
#define MODBUS_EVSE_STATUS_COUNT 12
#define MODBUS_EVSE_CONFIG_START 0x0100
#define MODBUS_EVSE_CONFIG_COUNT 10
#define MODBUS_SYS_CONFIG_START 0x0200
#define MODBUS_SYS_CONFIG_COUNT 27

#define MODBUS_MAX_REGISTER_READ MODBUS_SYS_CONFIG_COUNT
#define MODBUS_BUFFER_SIZE MODBUS_MAX_REGISTER_READ * 2 + 10

#define EMCUSTOM_ENDIANESS 0
#define EMCUSTOM_DATATYPE 0
#define EMCUSTOM_FUNCTION 4
#define EMCUSTOM_UREGISTER 0
#define EMCUSTOM_UDIVISOR 8
#define EMCUSTOM_IREGISTER 0
#define EMCUSTOM_IDIVISOR 8
#define EMCUSTOM_PREGISTER 0
#define EMCUSTOM_PDIVISOR 8
#define EMCUSTOM_EREGISTER 0
#define EMCUSTOM_EDIVISOR 8

#define ENDIANESS_LBF_LWF 0
#define ENDIANESS_LBF_HWF 1
#define ENDIANESS_HBF_LWF 2
#define ENDIANESS_HBF_HWF 3

// EVSE status
#define NODE_STATUS_STATE 64           // 0x0000: State
#define NODE_STATUS_ERROR 65           // 0x0001: Error
#define NODE_STATUS_CHARGECURRENT 66   // 0x0002: Charging current (A * 10)
#define NODE_STATUS_MODE 67            // 0x0003: EVSE Mode
#define NODE_STATUS_SOLAR_TIMER 68     // 0x0004: Solar Timer
#define NODE_STATUS_ACCESSBIT 69       // 0x0005: Access bit
#define NODE_STATUS_CONFIG_CHANGED 70  // 0x0006: Configuration changed
#define NODE_STATUS_CABLEMAX 71        // 0x0007: Maximum charging current (ReadOnly)
#define NODE_STATUS_PHASE_COUNT 72     // 0x0008: Number of used phases (ReadOnly) (not implemented)
#define NODE_STATUS_REAL_CURRENT 73    // 0x0009: Real charging current (ReadOnly) (not implemented)
#define NODE_STATUS_TEMP 74            // 0x000A: Temperature (ReadOnly)
// #define STATUS_SERIAL 75         // 0x000B: Serial number (ReadOnly)

// SW set to 0, set to output (driven low)
#define ONEWIRE_LOW                   \
    {                                 \
        digitalWrite(PIN_SW_IN, LOW); \
        pinMode(PIN_SW_IN, OUTPUT);   \
    }
// SW set to 1, set to output (driven high)
#define ONEWIRE_HIGH                   \
    {                                  \
        digitalWrite(PIN_SW_IN, HIGH); \
        pinMode(PIN_SW_IN, OUTPUT);    \
    }
// SW input (floating high)
#define ONEWIRE_FLOATHIGH pinMode(PIN_SW_IN, INPUT_PULLUP);

#define EVMETER_RESET_KWH_OFF 0
#define EVMETER_RESET_KWH_ON 1
#define EVMETER_RESET_KWH_POWERUP 2

#define NR_EVSES CLUSTER_NUM_EVSES

#define WORKFLOW_FINISHED 0
#define WORKFLOW_SKIP 99

#define WORKFLOW_SOLARSMART_REQUESTPVREADINGS 1
#define WORKFLOW_SOLARSMART_REQUESTMAINSREADINGS 2
#define WORKFLOW_SOLARSMART_FINDNEXTONLINESMARTEVSE 3
#define WORKFLOW_SOLARSMART_REQUESTEVENERGY 4
#define WORKFLOW_SOLARSMART_REQUESTEVPOWER 5

#define WORKFLOW_NORMAL_REQUESTREADINGSNODE1 6
#define WORKFLOW_NORMAL_REQUESTREADINGSNODE2 7
#define WORKFLOW_NORMAL_REQUESTREADINGSNODE3 8
#define WORKFLOW_NORMAL_REQUESTREADINGSNODE4 9
#define WORKFLOW_NORMAL_REQUESTREADINGSNODE5 10
#define WORKFLOW_NORMAL_REQUESTREADINGSNODE6 11
#define WORKFLOW_NORMAL_REQUESTREADINGSNODE7 12

#define WORKFLOW_NORMAL_PROCESSREADINGSNODE1 13
#define WORKFLOW_NORMAL_PROCESSREADINGSNODE2 14
#define WORKFLOW_NORMAL_PROCESSREADINGSNODE3 15
#define WORKFLOW_NORMAL_PROCESSREADINGSNODE4 16
#define WORKFLOW_NORMAL_PROCESSREADINGSNODE5 17
#define WORKFLOW_NORMAL_PROCESSREADINGSNODE6 18
#define WORKFLOW_NORMAL_PROCESSREADINGSNODE7 19

typedef enum mb_datatype {
    MB_DATATYPE_INT32 = 0,
    MB_DATATYPE_FLOAT32 = 1,
    MB_DATATYPE_INT16 = 2,
    MB_DATATYPE_MAX,
} MBDataType;

struct NodeStatus {
    bool Online;
    uint8_t ConfigChanged;
    uint8_t EVMeter;
    uint8_t EVAddress;
    uint8_t MinCurrent;  // Unused 0.1A
    uint8_t Phases;      // Unused
    uint16_t Timer;      // Unused (in seconds)
};

struct EMstruct {
    uint8_t Endianness;   // 0: low byte first, low word first, 1: low byte first,
                          // high word first, 2: high byte first, low word first,
                          // 3: high byte first, high word first
    uint8_t Function;     // 3: holding registers, 4: input registers
    MBDataType DataType;  // How data is represented on this Modbus meter
    uint16_t URegister;   // Single phase voltage (V)
    uint8_t UDivisor;     // 10^x
    uint16_t IRegister;   // Single phase current (A)
    uint8_t IDivisor;     // 10^x
    uint16_t PRegister;   // Total power (W) -- only used for EV/PV meter momentary
                          // power
    uint8_t PDivisor;     // 10^x
    uint16_t ERegister;   // Total energy (kWh)
    uint8_t EDivisor;     // 10^x
};

struct ModBus {
    uint8_t Address;
    uint8_t Function;
    uint16_t Register;
    uint16_t RegisterCount;
    uint16_t Value;
    uint8_t* Data;
    uint8_t DataLength;
    uint8_t Type;
    uint8_t RequestAddress;
    uint8_t RequestFunction;
    uint16_t RequestRegister;
    uint8_t Exception;
};

class EVSEModbus {
   public:
    EVSEModbus() {};

    void setup();
    void loop();
    void configureModbusMode(uint8_t newmode);

    void broadcastSysConfigToNodes(uint16_t values[]);
    void broadcastErrorFlagsToWorkerNodes(uint16_t flags);
    void broadcastSolarStopTimerToNodes(uint16_t value);
    void broadcastControllerModeToNodes(uint16_t newMode);
    void broadcastMasterBalancedCurrent(uint16_t balancedChargeCurrent[]);
    void sendNewStatusToNode(uint8_t nodeNr, uint16_t values[]);

    void updateSettings();
    void resetSettings();
    void evMeterResetKwhOnCharging();
    void setPvMeter(uint8_t value);
    void evMeterResetKwhOnStandby();

    int32_t getEvMeterEnergy() { return evMeterEnergy; };

    // Type of PV electric meter (0: Disabled / Constants EM_*)
    uint8_t pvMeter = PV_METER_DISABLED;
    uint8_t pvMeterAddress = PV_METER_ADDRESS;
    // Type of EV electric meter (0: Disabled / Constants EM_*)
    uint8_t evMeter = EV_METER_DISABLED;
    uint8_t evMeterAddress = EV_METER_ADDRESS;

    uint8_t grid = GRID_3WIRE;
    uint8_t mainsMeter = MAINS_METER_DISABLED;
    uint8_t mainsMeterAddress = MAINS_METER_ADDRESS;
    // What does Mains electric meter measure (0: Mains (Home+EVSE+PV) / 1:
    // Home+EVSE / 2: Home)
    uint8_t mainsMeterMeasure = MAINS_METER_MEASURE;
    // kWh meter value energy charged. (Wh) (will reset if state changes from
    // A->B)
    int32_t energyCharged = 0;
    // Measured Charge power in Watt by kWh meter
    int32_t powerMeasured = 0;

    struct EMstruct EMConfig[MM_CUSTOM + 1] = {
        /* DESC,      ENDIANNESS,      FCT, DATATYPE,            U_REG,DIV,
           I_REG,DIV, P_REG,DIV, E_REG,DIV */
        {/*"Disabled", */ ENDIANESS_LBF_LWF, 0, MB_DATATYPE_INT32, 0, 0, 0, 0, 0, 0, 0, 0},
        // Sensorbox (Own routine for request/receive)
        {/*"Sensorbox", */ ENDIANESS_HBF_HWF, 4, MB_DATATYPE_FLOAT32, 0xFFFF, 0, 0, 0, 0xFFFF, 0, 0xFFFF, 0},
        // PHOENIX CONTACT EEM-350-D-MCB (0,1V / mA / 0,1W / 0,1kWh) max read
        // count 11
        {/*"Phoenix C", */ ENDIANESS_HBF_LWF, 4, MB_DATATYPE_INT32, 0x0, 1, 0xC, 3, 0x28, 1, 0x3E, 1},
        // Finder 7E.78.8.400.0212 (V / A / W / Wh) max read count 127
        {/*"Finder", */ ENDIANESS_HBF_HWF, 4, MB_DATATYPE_FLOAT32, 0x1000, 0, 0x100E, 0, 0x1026, 0, 0x1106, 3},
        // Eastron SDM630 (V / A / W / kWh) max read count 80
        {/*"Eastron", */ ENDIANESS_HBF_HWF, 4, MB_DATATYPE_FLOAT32, 0x0, 0, 0x6, 0, 0x34, 0, 0x156, 0},
        // ABB B23 212-100 (0.1V / 0.01A / 0.01W / 0.01kWh) RS485 wiring reversed
        // / max read count 125
        {/*"ABB", */ ENDIANESS_HBF_HWF, 3, MB_DATATYPE_INT32, 0x5B00, 1, 0x5B0C, 2, 0x5B14, 2, 0x5002, 2},
        // SolarEdge SunSpec (0.01V (16bit) / 0.1A (16bit) / 1W  (16bit) / 1 Wh
        // (32bit))
        {/*"SolarEdge", */ ENDIANESS_HBF_HWF, 3, MB_DATATYPE_INT16, 40196, 0, 40191, 0, 40083, 0, 40226, 3},
        // WAGO 879-30x0 (V / A / kW / kWh)
        {/*"WAGO", */ ENDIANESS_HBF_HWF, 3, MB_DATATYPE_FLOAT32, 0x5002, 0, 0x500C, 0, 0x5012, 3, 0x6000, 0},
        {/*"Custom", */ ENDIANESS_LBF_LWF, 4, MB_DATATYPE_INT32, 0, 0, 0, 0, 0, 0, 0, 0}};

   protected:
    static void modbusOnClientError(Error error, uint32_t token);
    static void modbusOnClientData(ModbusMessage msg, uint32_t token);
    static ModbusMessage onModbusNodeRequest(ModbusMessage request);
    static ModbusMessage onModbusServerbroadcast(ModbusMessage msg);
    static ModbusMessage onModbusMainsMeterResponse(ModbusMessage msg);
    static ModbusMessage onModbusEVMeterResponse(ModbusMessage msg);
    static ModbusMessage onModbusPVMeterResponse(ModbusMessage msg);

   private:
    ModbusClientRTU* MBclient = NULL;
    struct ModBus MB;

    void readEpromSettings();
    void writeEpromSettings();
    void validateSettings();

    // NodeNr (1-7)
    void requestNodeConfig(uint8_t NodeNr);
    void receiveNodeConfig(uint8_t* buf, uint8_t NodeNr);
    void requestNodeStatus(uint8_t NodeNr);
    void receiveNodeStatus(uint8_t* buf, uint8_t NodeNr);
    void modbusClientDataHandler(ModbusMessage msg, uint32_t token);
    ModbusMessage modbusNodeRequestHandler(ModbusMessage request);
    void modbusBbroadcastHandler(ModbusMessage msg);
    void modbusMainsMeterResponseHandler(ModbusMessage msg);
    void modbusEVMeterResponseHandler(ModbusMessage msg);
    void modbusPVMeterResponseHandler(ModbusMessage msg);
    uint8_t mapModbusRegister2MenuItemID();

    void modbusWorkflow();
    void modbusWorkflowNormalMode();
    void modbusWorkflowSolarSmartMode();

    // last CT communication, to calculate timeout
    unsigned long lastCTRequestMillis = 0;
    // Flag to request Modbus information
    uint8_t workflowModbusRequest = WORKFLOW_FINISHED;
    uint8_t workflowPollingEVNodeNumber = NR_EVSES;

    // Create a ModbusRTU server and client instance on Serial1
    // TCP timeout set to 2000 ms
    ModbusServerRTU* MBserver = NULL;

    // if set, reset EV kwh meter at state transition B->C . Cleared when
    // charging, reset to 1 when disconnected (state A)
    uint8_t evMeterResetKwh = EVMETER_RESET_KWH_POWERUP;
    // kWh meter value is stored once EV is connected to EVSE (Wh)
    int32_t evMeterEnergyAtStart = 0;
    int32_t evMeterEnergy = 0;

    // Used by SmartEVSE fuctions
    int32_t CM[3] = {0, 0, 0};
    int32_t PV[3] = {0, 0, 0};
    char sprintfStr[128];

    struct NodeStatus Node[NR_EVSES] = {
        // 0: Master / 1: Node 1 ...
        /*         Config   EV     EV       Min                    *
         * Online, Changed, Meter, Address, Current, Phases, Timer */
        {true, 0, 0, 0, 0, 0, 0},  {false, 1, 0, 0, 0, 0, 0}, {false, 1, 0, 0, 0, 0, 0}, {false, 1, 0, 0, 0, 0, 0},
        {false, 1, 0, 0, 0, 0, 0}, {false, 1, 0, 0, 0, 0, 0}, {false, 1, 0, 0, 0, 0, 0}, {false, 1, 0, 0, 0, 0, 0}};

    // ########################### Modbus main functions
    // ###########################

    void ModbusReadInputRequest(uint8_t address, uint8_t function, uint16_t reg, uint16_t quantity);
    void ModbusWriteSingleRequest(uint8_t address, uint16_t reg, uint16_t value);
    void ModbusWriteMultipleRequest(uint8_t address, uint16_t reg, uint16_t* values, uint8_t count);
    void ModbusDecode(uint8_t* buf, uint8_t len);

    // ########################### EVSE modbus functions
    // ###########################

    void requestMeasurement(uint8_t Meter, uint8_t Address, uint16_t Register, uint8_t Count);
    signed int receiveMeasurement(uint8_t* buf,
                                  uint8_t pos,
                                  uint8_t Endianness,
                                  MBDataType dataType,
                                  signed char Divisor);
    void requestEnergyMeasurement(uint8_t Meter, uint8_t Address);
    signed int receiveEnergyMeasurement(uint8_t* buf, uint8_t Meter);
    void requestPowerMeasurement(uint8_t Meter, uint8_t Address);
    signed int receivePowerMeasurement(uint8_t* buf, uint8_t Meter);
    void requestCurrentMeasurement(uint8_t Meter, uint8_t Address);
    uint8_t receiveCurrentMeasurement(uint8_t* buf, uint8_t Meter, signed int* var);

    void ModbusSend8(uint8_t address, uint8_t function, uint16_t reg, uint16_t data);
    void combineBytes(void* var, uint8_t* buf, uint8_t pos, uint8_t endianness, MBDataType dataType);
};

extern EVSEModbus evseModbus;

#endif