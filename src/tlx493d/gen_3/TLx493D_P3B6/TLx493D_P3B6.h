#ifndef TLX493D_P3B6_H
#define TLX493D_P3B6_H

// std includes
#include <stdbool.h>
#include <stdint.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"
// #include "tlx493d_common.h"

// common to same generation of sensors
// #include "tlx493d_gen_3_common.h"

// sensor specific includes


#ifdef __cplusplus

extern "C" {

#endif


bool TLx493D_P3B6_init(TLx493D_t *sensor);
bool TLx493D_P3B6_deinit(TLx493D_t *sensor);

bool TLx493D_P3B6_readRegisters(TLx493D_t *sensor);


void TLx493D_P3B6_calculateRawTemperature(TLx493D_t *sensor, uint16_t *temperature);
bool TLx493D_P3B6_getRawTemperature(TLx493D_t *sensor, uint16_t *temperature);

void TLx493D_P3B6_calculateRawMagneticField(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z);
bool TLx493D_P3B6_getRawMagneticField(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z);

void TLx493D_P3B6_calculateRawMagneticFieldAndTemperature(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z, uint16_t *temperature);
bool TLx493D_P3B6_getRawMagneticFieldAndTemperature(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z, uint16_t *temperature);

void TLx493D_P3B6_calculateTemperature(TLx493D_t *sensor, double *temp);
bool TLx493D_P3B6_getTemperature(TLx493D_t *sensor, double *temp);

void TLx493D_P3B6_calculateMagneticField(TLx493D_t *sensor, double *x, double *y, double *z);
bool TLx493D_P3B6_getMagneticField(TLx493D_t *sensor, double *x, double *y, double *z);

void TLx493D_P3B6_calculateMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp);
bool TLx493D_P3B6_getMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp);


bool TLx493D_P3B6_setDefaultConfig(TLx493D_t *sensor);

bool TLx493D_P3B6_enable1ByteMode(TLx493D_t *sensor);

void TLx493D_P3B6_setResetValues(TLx493D_t *sensor);

void TLx493D_P3B6_calculateRawMagneticFieldAtTemperature(TLx493D_t *sensor, int16_t *rawTemp, TLx493D_SensitivityType_t sens, double mT, int16_t *rawMF);

void TLx493D_P3B6_getSensitivityScaleFactor(TLx493D_t *sensor, double *sf);


#ifdef __cplusplus

}

#endif


#endif // TLX493D_P3B6_H
