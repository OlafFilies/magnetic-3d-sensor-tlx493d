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

/**
 * Utility to concatenate MSB and LSB of field values
 */
void gen_2_concatBytes(Sensor_ts *sensor, Register_ts *msb, Register_ts *lsb, int16_t *result);

void gen_2_getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue);
void gen_2_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue);

bool gen_2_writeRegister(Sensor_ts* sensor, uint8_t bitField);
bool gen_2_readRegisters(Sensor_ts *sensor);

uint8_t gen_2_calculateFuseParityBit(Sensor_ts *sensor);
uint8_t gen_2_calculateConfigurationParityBit(Sensor_ts *sensor);
uint8_t gen_2_calculateBusParityBit(Sensor_ts *sensor);

bool gen_2_hasValidData(Sensor_ts *sensor);
bool gen_2_isFunctional(Sensor_ts *sensor);
bool gen_2_hasValidBusParity(Sensor_ts *sensor);
bool gen_2_hasValidFuseParity(Sensor_ts *sensor);
bool gen_2_hasValidConfigurationParity(Sensor_ts *sensor);
bool gen_2_hasValidTBit(Sensor_ts *sensor) ;
bool gen_2_hasValidPD3Bit(Sensor_ts *sensor);
bool gen_2_hasValidPD0Bit(Sensor_ts *sensor);
bool gen_2_setIICAddress(Sensor_ts *sensor, StandardIICAddresses_te addr);
uint8_t gen_2_getID(Sensor_ts *sensor);
uint8_t gen_2_getFrameCounter(Sensor_ts *sensor) ;
uint8_t gen_2_getType(Sensor_ts *sensor);
uint8_t gen_2_getHWV(Sensor_ts *sensor);


#endif // SENSORS_GEN_2_COMMON_H
