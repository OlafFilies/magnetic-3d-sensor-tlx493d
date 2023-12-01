#ifndef TLX493D_GEN_3_COMMON_H
#define TLX493D_GEN_3_COMMON_H


#include "tlx493d_types.h"


#ifdef __cplusplus

extern "C" {

#endif


bool tlx493d_gen_3_readRegistersSPI(TLx493D_t *sensor);

void tlx493d_gen_3_calculateRawTemperature(TLx493D_t *sensor, uint8_t tempMSBBF, uint8_t tempLSBBF, uint16_t *temperature);
void tlx493d_gen_3_calculateRawMagneticField(TLx493D_t *sensor, uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF,
                                             uint8_t bzMSBBF, uint8_t bzLSBBF, uint16_t *x, uint16_t *y, uint16_t *z);

void tlx493d_gen_3_calculateTemperature(TLx493D_t *sensor, uint8_t tempMSBBF, uint8_t tempLSBBF, double *temperature);
void tlx493d_gen_3_calculateMagneticField(TLx493D_t *sensor, uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF,
                                          uint8_t bzMSBBF, uint8_t bzLSBBF, uint8_t tempMSBBF, uint8_t tempLSBBF,
                                          double *x, double *y, double *z);

uint8_t tlx493d_gen_3_selectIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t addr);


#ifdef __cplusplus

}

#endif


#endif // TLX493D_GEN_3_COMMON_H
