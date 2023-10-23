#ifndef SENSORS_GEN_2_COMMON_H
#define SENSORS_GEN_2_COMMON_H

// std includes
#include <stdbool.h>


// project c includes
// common to all sensors
#include "sensor_types.h"

typedef enum {
    GEN_2_STD_IIC_ADDR_A0 = 0,
    GEN_2_STD_IIC_ADDR_A1 = 1,
    GEN_2_STD_IIC_ADDR_A2 = 2,
    GEN_2_STD_IIC_ADDR_A3 = 3
} StandardIICAddresses_te;


void gen_2_getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue);
void gen_2_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue);

bool gen_2_writeRegister(Sensor_ts* sensor, uint8_t bitField);
bool gen_2_readRegisters(Sensor_ts *sensor);

// TODO: cleanup
// uint8_t gen_2_calculateFuseParityBit(Sensor_ts *sensor);
// uint8_t gen_2_calculateBusParityBit(Sensor_ts *sensor);
uint8_t gen_2_calculateFuseParity(Sensor_ts *sensor);
uint8_t gen_2_calculateBusParity(Sensor_ts *sensor);

bool gen_2_hasValidData(Sensor_ts *sensor);

bool gen_2_hasValidTemperatureData(Sensor_ts *sensor);
bool gen_2_hasValidFieldData(Sensor_ts *sensor);

bool gen_2_isFunctional(Sensor_ts *sensor);
bool gen_2_hasValidBusParity(Sensor_ts *sensor);
bool gen_2_hasValidFuseParity(Sensor_ts *sensor);
bool gen_2_hasValidConfigurationParity(Sensor_ts *sensor);
bool gen_2_hasValidTBit(Sensor_ts *sensor) ;
bool gen_2_hasValidPD3Bit(Sensor_ts *sensor);
bool gen_2_hasValidPD0Bit(Sensor_ts *sensor);

bool gen_2_setPowerMode(Sensor_ts *sensor, uint8_t mode);
bool gen_2_setIICAddress(Sensor_ts *sensor, StandardIICAddresses_te addr);

bool gen_2_enableAngularMeasurement(Sensor_ts *sensor);
bool gen_2_disableAngularMeasurement(Sensor_ts *sensor);

bool gen_2_setTriggerBits(Sensor_ts *sensor, uint8_t bits);

uint8_t gen_2_getID(Sensor_ts *sensor);
uint8_t gen_2_getFrameCounter(Sensor_ts *sensor) ;
uint8_t gen_2_getType(Sensor_ts *sensor);
uint8_t gen_2_getHWV(Sensor_ts *sensor);

bool gen_2_hasValidIICadr(Sensor_ts *sensor, uint8_t id, uint8_t iicAdr);
bool gen_2_hasWakeup(Sensor_ts *sensor, uint8_t type);

#endif // SENSORS_GEN_2_COMMON_H