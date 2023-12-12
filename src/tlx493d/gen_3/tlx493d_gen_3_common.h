#ifndef TLX493D_GEN_3_COMMON_H
#define TLX493D_GEN_3_COMMON_H


#include "tlx493d_types.h"


#ifdef __cplusplus

extern "C" {

#endif


bool tlx493d_gen_3_readRegistersSPI(TLx493D_t *sensor);

void tlx493d_gen_3_calculateRawTemperature(TLx493D_t *sensor, uint8_t tempMSBBF, uint8_t tempLSBBF, int16_t *temperature);
void tlx493d_gen_3_calculateRawMagneticField(TLx493D_t *sensor, uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF,
                                             uint8_t bzMSBBF, uint8_t bzLSBBF, int16_t *x, int16_t *y, int16_t *z);

void tlx493d_gen_3_calculateTemperature(TLx493D_t *sensor, uint8_t tempMSBBF, uint8_t tempLSBBF, double *temperature);
void tlx493d_gen_3_calculateMagneticField(TLx493D_t *sensor, uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF,
                                          uint8_t bzMSBBF, uint8_t bzLSBBF, uint8_t tempMSBBF, uint8_t tempLSBBF,
                                          double *x, double *y, double *z);

uint8_t tlx493d_gen_3_selectIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t addr);

void tlx493d_gen_3_calculateRawMagneticFieldAtTemperature(TLx493D_t *sensor, int16_t rawTemp, TLx493D_SensitivityType_t sens,
                                                          double xInmT, double yInmT, double zInmT,
                                                          int16_t *x, int16_t *y, int16_t *z);

bool tlx493d_gen_3_setMeasurement(TLx493D_t *sensor, uint8_t channelBF, TLx493D_MeasurementType_t val);
bool tlx493d_gen_3_setTrigger(TLx493D_t *sensor, uint8_t trigBF, TLx493D_TriggerType_t val);
bool tlx493d_gen_3_setSensitivity(TLx493D_t *sensor, uint8_t shortBF, uint8_t xtrShortBF, TLx493D_SensitivityType_t val);

// TODO: same as for gen_2 => make common
bool tlx493d_gen_2_setThreshold(TLx493D_t *sensor, uint8_t msbsBF, uint8_t lsbsBF, int16_t threshold12Bits);
bool tlx493d_gen_3_setWakeUpThresholdsAsInteger(TLx493D_t *sensor,
                                                uint8_t xlMSBBF, uint8_t xlLSBBF, uint8_t xhMSBBF, uint8_t xhLSBBF,
                                                uint8_t ylMSBBF, uint8_t ylLSBBF, uint8_t yhMSBBF, uint8_t yhLSBBF,
                                                uint8_t zlMSBBF, uint8_t zlLSBBF, uint8_t zhMSBBF, uint8_t zhLSBBF,
                                                int16_t xlTh, int16_t xhTh, int16_t ylTh, int16_t yhTh, int16_t zlTh, int16_t zhTh);


bool tlx493d_gen_3_hasValidFuseParity(TLx493D_t *sensor, uint8_t fpfBF);

double tlx493d_gen_3_getSensitivityScaleFactor(TLx493D_t *sensor, TLx493D_AvailableSensitivityType_t sens, uint8_t x2BF, uint8_t x4BF);


#ifdef __cplusplus

}

#endif


#endif // TLX493D_GEN_3_COMMON_H
