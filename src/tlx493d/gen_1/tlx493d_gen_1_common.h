#ifndef TLX493D_GEN_1_COMMON_H
#define TLX493D_GEN_1_COMMON_H

// std includes
#include <stdbool.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"


#ifdef __cplusplus

extern "C" {

#endif


void tlx493d_gen_1_getBitfield(TLx493D_t *sensor, uint8_t bitField, uint8_t *bitFieldValue);
uint8_t tlx493d_gen_1_returnBitfield(TLx493D_t *sensor, uint8_t bitField);
void tlx493d_gen_1_setBitfield(TLx493D_t *sensor, uint8_t bitField, uint8_t newBitFieldValue);

bool tlx493d_gen_1_writeRegister(TLx493D_t* sensor, uint8_t bitField);

bool tlx493d_gen_1_readRegisters(TLx493D_t *sensor);

bool tlx493d_gen_1_hasValidFuseParity(TLx493D_t *sensor, uint8_t ffBF);
bool tlx493d_gen_1_isFunctional(TLx493D_t *sensor, uint8_t ffBF);
bool tlx493d_gen_1_hasValidTBit(TLx493D_t *sensor, uint8_t tBF);
bool tlx493d_gen_1_hasValidPDBit(TLx493D_t *sensor, uint8_t pdBF);
bool tlx493d_gen_1_hasValidData(TLx493D_t *sensor, uint8_t tBF, uint8_t pdBF);


#ifdef __cplusplus

}

#endif


#endif // TLX493D_GEN_1_COMMON_H
