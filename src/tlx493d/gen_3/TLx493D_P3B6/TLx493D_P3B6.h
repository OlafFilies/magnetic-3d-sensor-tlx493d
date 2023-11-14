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

void TLx493D_P3B6_calculateTemperature(TLx493D_t *sensor, double *temp);
bool TLx493D_P3B6_getTemperature(TLx493D_t *sensor, double *temp);

void TLx493D_P3B6_calculateMagneticField(TLx493D_t *sensor, double *x, double *y, double *z);
bool TLx493D_P3B6_getMagneticField(TLx493D_t *sensor, double *x, double *y, double *z);

void TLx493D_P3B6_calculateMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp);
bool TLx493D_P3B6_getMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp);


bool TLx493D_P3B6_setDefaultConfig(TLx493D_t *sensor);

bool TLx493D_P3B6_enable1ByteMode(TLx493D_t *sensor);

void TLx493D_P3B6_setResetValues(TLx493D_t *sensor);


#ifdef __cplusplus

}

#endif


#endif // TLX493D_P3B6_H
