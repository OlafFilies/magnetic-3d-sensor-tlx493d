// std includes
#include <stdbool.h>
#include <stddef.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"
#include "tlx493d_common_defines.h"
#include "tlx493d_common.h"
#include "Logger.h"

// common to same generation of sensors
#include "tlx493d_gen_2_common_defines.h"
#include "tlx493d_gen_2_common.h"

#include "tlx493d_gen_3_common_defines.h"
#include "tlx493d_gen_3_common.h"


bool tlx493d_gen_3_readRegistersSPI(TLx493D_ts *sensor) {
    // sensor->regMap[0] = GEN_3_SPI_READ_BIT_ON | GEN_3_SPI_AUTO_INC_BIT_OFF;
    return transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
}


void tlx493d_gen_3_calculateRawTemperature(TLx493D_ts *sensor, double *temp, uint8_t tempMSBBF, uint8_t tempLSBBF) {
    int16_t value = 0;

    tlx493d_common_concatBytes(sensor, tempMSBBF, tempLSBBF, &value);
    *temp = (double) value;
}


void tlx493d_gen_3_calculateTemperature(TLx493D_ts *sensor, double *temp, uint8_t tempMSBBF, uint8_t tempLSBBF) {
    int16_t value = 0;

    tlx493d_common_concatBytes(sensor, tempMSBBF, tempLSBBF, &value);
    *temp = (((double) value - GEN_3_TEMP_OFFSET) / GEN_3_TEMP_MULT) + GEN_3_TEMP_REF;
}


void tlx493d_gen_3_calculateMagneticField(TLx493D_ts *sensor, double *x, double *y, double *z,
                                          uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF,
                                          uint8_t bzMSBBF, uint8_t bzLSBBF, uint8_t tempMSBBF, uint8_t tempLSBBF) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    tlx493d_common_concatBytes(sensor, bxMSBBF, bxLSBBF, &valueX);
    tlx493d_common_concatBytes(sensor, byMSBBF, byLSBBF, &valueY);
    tlx493d_common_concatBytes(sensor, bzMSBBF, bzLSBBF, &valueZ);

// // TODO: temp unten in LSB format; x,y,z dann auch in LSB format => mittels sensitivity umrechnen in mT !
// // TODO: TLx493D_P3I8_calculateTemperatureRaw hinzufügen um die LSB Werte für Temp zu bekommen !
double r    = 1.0; // TODO: get factor from registers : full, double, quadruple  ; r is range specific !
// double temp = 0.0;
double temp = 0.0;
double sensitivity = GEN_3_FULL_RANGE_FIELD_SENSITIVITY; // TODO: r is range specific !
    
    tlx493d_gen_3_calculateRawTemperature(sensor, &temp, tempMSBBF, tempLSBBF);

    *x = (r * (GEN_3_O0x + temp * (GEN_3_O1x + temp * (GEN_3_O2x + GEN_3_O3x * temp)))
          + ((double) valueX) * (GEN_3_L0x + temp * (GEN_3_L1x + temp * (GEN_3_L2x + GEN_3_L3x * temp))))
       / sensitivity;

    *y = (r * (GEN_3_O0y + temp * (GEN_3_O1y + temp * (GEN_3_O2y + GEN_3_O3y * temp)))
          + ((double) valueY) * (GEN_3_L0y + temp * (GEN_3_L1y + temp * (GEN_3_L2y + GEN_3_L3y * temp))))
       / sensitivity;

    *z = (r * (GEN_3_O0z + temp * (GEN_3_O1z + temp * (GEN_3_O2z + GEN_3_O3z * temp)))
          + ((double) valueZ) * (GEN_3_L0z + temp * (GEN_3_L1z + temp * (GEN_3_L2z + GEN_3_L3z * temp))))
       / sensitivity;
}

// Calc. int from mT :
// valueX = x * sensitivity;
// valueY = y * sensitivity;
// valueZ = z * sensitivity;


// void tlx493d_gen_3_calculateMagneticFieldAndTemperature(TLx493D_ts *sensor, double *x, double *y, double *z, double *temp,
//                                                         uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF,
//                                                         uint8_t bzMSBBF, uint8_t bzLSBBF, uint8_t tempMSBBF, uint8_t tempLSBBF) {

//     tlx493d_gen_3_calculateMagneticField(sensor, x, y, z, bxMSBBF, bxLSBBF, byMSBBF, byLSBBF, bzMSBBF, bzLSBBF);
//     tlx493d_gen_3_calculateTemperature(sensor, temp, tempMSBBF, tempLSBBF);
// }
