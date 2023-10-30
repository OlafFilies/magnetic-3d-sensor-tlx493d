#ifndef TLX493D_GEN_3_COMMON_H
#define TLX493D_GEN_3_COMMON_H


#include "tlx493d_types.h"


// void tlx493d_gen_3_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue);


// bool tlx493d_gen_3_writeRegisterSPI(Sensor_ts* sensor, uint8_t bitField);
bool tlx493d_gen_3_readRegistersSPI(Sensor_ts *sensor);


#endif // TLX493D_GEN_3_COMMON_H
