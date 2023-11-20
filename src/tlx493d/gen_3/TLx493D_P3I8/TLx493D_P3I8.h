#ifndef TLX493D_P3I8_H
#define TLX493D_P3I8_H

// std includes
#include <stdbool.h>
#include <stdint.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"

// common to same generation of sensors

// sensor specific includes


#ifdef __cplusplus

extern "C" {

#endif


bool TLx493D_P3I8_init(TLx493D_t *sensor);
bool TLx493D_P3I8_deinit(TLx493D_t *sensor);

bool TLx493D_P3I8_readRegisters(TLx493D_t *sensor);


void TLx493D_P3I8_calculateRawTemperature(TLx493D_t *sensor, uint16_t *temperature);
bool TLx493D_P3I8_getRawTemperature(TLx493D_t *sensor, uint16_t *temperature);

void TLx493D_P3I8_calculateRawMagneticField(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z);
bool TLx493D_P3I8_getRawMagneticField(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z);

void TLx493D_P3I8_calculateRawMagneticFieldAndTemperature(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z, uint16_t *temperature);
bool TLx493D_P3I8_getRawMagneticFieldAndTemperature(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z, uint16_t *temperature);


void TLx493D_P3I8_calculateTemperature(TLx493D_t *sensor, double *temp);
bool TLx493D_P3I8_getTemperature(TLx493D_t *sensor, double *temp);

void TLx493D_P3I8_calculateMagneticField(TLx493D_t *sensor, double *x, double *y, double *z);
bool TLx493D_P3I8_getMagneticField(TLx493D_t *sensor, double *x, double *y, double *z);

void TLx493D_P3I8_calculateMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp);
bool TLx493D_P3I8_getMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp);


bool TLx493D_P3I8_setDefaultConfig(TLx493D_t *sensor);

void TLx493D_P3I8_setResetValues(TLx493D_t *sensor);

uint8_t TLx493D_P3I8_selectIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t addr);

void TLx493D_P3I8_calculateRawMagneticFieldAtTemperature(TLx493D_t *sensor, int16_t rawTemp, TLx493D_SensitivityType_t sens, double mT, int16_t *rawMF);

void TLx493D_P3I8_getSensitivityScaleFactor(TLx493D_t *sensor, double *sf);


#ifdef __cplusplus

}

#endif


#endif // TLX493D_P3I8_H
