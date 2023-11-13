#ifndef TLX493D_P2B6_H
#define TLX493D_P2B6_H

/** std includes*/
#include <stdbool.h>
#include <stdint.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"
// #include "tlx493d_common.h"

/** Common to the same generation of senors */
// #include "tlx493d_gen_2_common.h"

// sensor specific includes


bool TLx493D_P2B6_init(TLx493D_t *sensor);
bool TLx493D_P2B6_deinit(TLx493D_t *sensor);

bool TLx493D_P2B6_readRegisters(TLx493D_t *sensor);

void TLx493D_P2B6_calculateTemperature(TLx493D_t *sensor, double *temp);
bool TLx493D_P2B6_getTemperature(TLx493D_t *sensor, double *temp);

void TLx493D_P2B6_calculateMagneticField(TLx493D_t *sensor, double *x, double *y, double *z);
bool TLx493D_P2B6_getMagneticField(TLx493D_t *sensor, double *x, double *y, double *z);

void TLx493D_P2B6_calculateMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp);
bool TLx493D_P2B6_getMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp);

bool TLx493D_P2B6_setDefaultConfig(TLx493D_t *sensor);

bool TLx493D_P2B6_enable1ByteMode(TLx493D_t *sensor);
bool TLx493D_P2B6_disable1ByteMode(TLx493D_t *sensor);

uint8_t TLx493D_P2B6_calculateConfigurationParity(TLx493D_t *sensor);

void TLx493D_P2B6_setResetValues(TLx493D_t *sensor);


#endif // TLX493D_P2B6_H
