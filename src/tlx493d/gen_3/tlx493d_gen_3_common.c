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


bool tlx493d_gen_3_readRegistersSPI(TLx493D_t *sensor) {
    return transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
}


void tlx493d_gen_3_calculateRawTemperature(TLx493D_t *sensor, uint8_t tempMSBBF, uint8_t tempLSBBF, uint16_t *temperature) {
    tlx493d_common_calculateRawTemperature(sensor, tempMSBBF, tempLSBBF, temperature);
}


void tlx493d_gen_3_calculateRawMagneticField(TLx493D_t *sensor, uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF,
                                             uint8_t bzMSBBF, uint8_t bzLSBBF, uint16_t *x, uint16_t *y, uint16_t *z) {
    tlx493d_common_calculateRawMagneticField(sensor, bxMSBBF, bxLSBBF, byMSBBF, byLSBBF, bzMSBBF, bzLSBBF, x, y, z);
}


void tlx493d_gen_3_calculateTemperature(TLx493D_t *sensor, uint8_t tempMSBBF, uint8_t tempLSBBF, double *temperature) {
    int16_t value = 0;

    tlx493d_common_concatBytes(sensor, tempMSBBF, tempLSBBF, &value);
    *temperature = (((double) value - GEN_3_TEMP_OFFSET) / GEN_3_TEMP_MULT) + GEN_3_TEMP_REF;
}


// TODO: cleanup and check code for short and xtra short ranges 1
void tlx493d_gen_3_calculateMagneticField(TLx493D_t *sensor, uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF,
                                          uint8_t bzMSBBF, uint8_t bzLSBBF, uint8_t tempMSBBF, uint8_t tempLSBBF,
                                          double *x, double *y, double *z) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    tlx493d_common_concatBytes(sensor, bxMSBBF, bxLSBBF, &valueX);
    tlx493d_common_concatBytes(sensor, byMSBBF, byLSBBF, &valueY);
    tlx493d_common_concatBytes(sensor, bzMSBBF, bzLSBBF, &valueZ);

// // TODO: temp unten in LSB format; x,y,z dann auch in LSB format => mittels sensitivity umrechnen in mT !
// // TODO: TLx493D_P3I8_calculateTemperatureRaw hinzufügen um die LSB Werte für Temp zu bekommen !


double sf;
sensor->functions->getSensitivityScaleFactor(sensor, &sf);

// double sensitivity = GEN_3_FULL_RANGE_FIELD_SENSITIVITY;
uint16_t rawTemp;

    tlx493d_gen_3_calculateRawTemperature(sensor, tempMSBBF, tempLSBBF, &rawTemp);
double temp = (double) rawTemp;

    *x = ((GEN_3_O0x + temp * (GEN_3_O1x + temp * (GEN_3_O2x + temp * GEN_3_O3x)))
          + ((double) valueX) * (GEN_3_L0x + temp * (GEN_3_L1x + temp * (GEN_3_L2x + temp * GEN_3_L3x))) / sf)
       / GEN_3_FULL_RANGE_FIELD_SENSITIVITY;

    *y = ((GEN_3_O0y + temp * (GEN_3_O1y + temp * (GEN_3_O2y + temp * GEN_3_O3y)))
          + ((double) valueY) * (GEN_3_L0y + temp * (GEN_3_L1y + temp * (GEN_3_L2y + temp * GEN_3_L3y))) / sf)
       / GEN_3_FULL_RANGE_FIELD_SENSITIVITY;

    *z = ((GEN_3_O0z + temp * (GEN_3_O1z + temp * (GEN_3_O2z + temp * GEN_3_O3z)))
          + ((double) valueZ) * (GEN_3_L0z + temp * (GEN_3_L1z + temp * (GEN_3_L2z + temp * GEN_3_L3z))) / sf)
       / GEN_3_FULL_RANGE_FIELD_SENSITIVITY;


// double r    = 1.0; // TODO: get factor from registers : full, double, quadruple  ; r is range specific !
    // *x = (r * (GEN_3_O0x + temp * (GEN_3_O1x + temp * (GEN_3_O2x + temp * GEN_3_O3x)))
    //       + ((double) valueX) * (GEN_3_L0x + temp * (GEN_3_L1x + temp * (GEN_3_L2x + temp * GEN_3_L3x))))
    //    / sensitivity;

    // *y = (r * (GEN_3_O0y + temp * (GEN_3_O1y + temp * (GEN_3_O2y + temp * GEN_3_O3y)))
    //       + ((double) valueY) * (GEN_3_L0y + temp * (GEN_3_L1y + temp * (GEN_3_L2y + temp * GEN_3_L3y))))
    //    / sensitivity;

    // *z = (r * (GEN_3_O0z + temp * (GEN_3_O1z + temp * (GEN_3_O2z + temp * GEN_3_O3z)))
    //       + ((double) valueZ) * (GEN_3_L0z + temp * (GEN_3_L1z + temp * (GEN_3_L2z + temp * GEN_3_L3z))))
    //    / sensitivity;
}

// Calc. int from mT :
// valueX = x * sensitivity;
// valueY = y * sensitivity;
// valueZ = z * sensitivity;


uint8_t tlx493d_gen_3_selectIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t addr) {
    switch(addr) {
        case TLx493D_IIC_ADDR_A0_e : return GEN_3_STD_IIC_ADDR_WRITE_A0;

        case TLx493D_IIC_ADDR_A1_e : return GEN_3_STD_IIC_ADDR_WRITE_A1;

        case TLx493D_IIC_ADDR_A2_e : return GEN_3_STD_IIC_ADDR_WRITE_A2;

        case TLx493D_IIC_ADDR_A3_e : return GEN_3_STD_IIC_ADDR_WRITE_A3;

        default : errorSelectionNotSupportedForSensorType(sensor, addr, "TLx493D_IICAddressType_t");
                  return 0;
    }
}
