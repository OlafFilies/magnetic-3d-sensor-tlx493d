#ifndef TLX493D_GEN_2_COMMON_H
#define TLX493D_GEN_2_COMMON_H

// std includes
#include <stdbool.h>


// project c includes
// common to all sensors
#include "tlx493d_types.h"


typedef enum {
    GEN_2_STD_IIC_ADDR_A0 = 0,
    GEN_2_STD_IIC_ADDR_A1 = 1,
    GEN_2_STD_IIC_ADDR_A2 = 2,
    GEN_2_STD_IIC_ADDR_A3 = 3
} StandardIICAddresses_te;


// void tlx493d_gen_2_getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue);
// void tlx493d_gen_2_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue);

// bool tlx493d_gen_2_writeRegister(Sensor_ts* sensor, uint8_t bitField);
// bool tlx493d_gen_2_readRegisters(Sensor_ts *sensor);
bool tlx493d_gen_2_setDefaultConfig(Sensor_ts *sensor, uint8_t configREG, uint8_t mod1REG, uint8_t mod2REG, uint8_t cpBF, uint8_t caBF, uint8_t intBF);

// bool setPowerMode(Sensor_ts *sensor, enum <possible combinations> mode);  // value of mode is sensor / generation specific !
bool tlx493d_gen_2_setPowerMode(Sensor_ts *sensor, uint8_t modeBF, uint8_t fpBF, uint8_t mode);

// bool setIICAddress(Sensor_ts *sensor, enum <possible combinations> addr); // Gen. 1 and 2
bool tlx493d_gen_2_setIICAddress(Sensor_ts *sensor, uint8_t iicadrBF, uint8_t fpBF, StandardIICAddresses_te addr);

void tlx493d_gen_2_calculateTemperature(Sensor_ts *sensor, double *temp, uint8_t tempMSBBF, uint8_t tempLSBBF);
void tlx493d_gen_2_calculateMagneticField(Sensor_ts *sensor, double *x, double *y, double *z,
                                  uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF, uint8_t bzMSBBF, uint8_t bzLSBBF);
void tlx493d_gen_2_calculateMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp,
                                                uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF, uint8_t bzMSBBF, uint8_t bzLSBBF,
                                                uint8_t tempMSBBF, uint8_t tempLSBBF);                

// bool selectMeasuredValues(Sensor_ts *sensor, enum <possible combinations> mVals); // Bx/By/Bz, Bx/By, Bx/By/Temp, ...
// or do it separately ? Like :
bool tlx493d_gen_2_setDisableTemperatureMeasurement(Sensor_ts *sensor, uint8_t dtBF, uint8_t cpBF, uint8_t dt);
bool tlx493d_gen_2_setAngularMeasurement(Sensor_ts *sensor, uint8_t amBF, uint8_t dtBF, uint8_t cpBF, uint8_t am, uint8_t dt);

// value of update rate is sensor / generation specific !
// bool setUpdateRate(Sensor_ts *sensor, enum <possible combinations> rate);
bool tlx493d_gen_2_setUpdateRate(Sensor_ts *sensor, uint8_t fpBF, uint8_t prdBF, uint8_t ur);
bool tlx493d_gen_2_setShortRangeSensitivity(Sensor_ts *sensor, uint8_t x2BF, uint8_t cpBF, uint8_t srs);

// bool setInterruptAndCollisionAvoidance(Sensor_ts *sensor, enum <possible combinations> eVal);
bool tlx493d_gen_2_setInterruptAndCollisionAvoidance(Sensor_ts *sensor, uint8_t intBF, uint8_t caBF, uint8_t fpBF, uint8_t prdBF, bool intIsOn, bool caIsOn);

bool tlx493d_gen_2_setTrigger(Sensor_ts *sensor, uint8_t trigBF, uint8_t cpBF, uint8_t trig);
bool tlx493d_gen_2_setTriggerBits(Sensor_ts *sensor, uint8_t trigBF, uint8_t cpBF, uint8_t bits);

bool tlx493d_gen_2_setMagneticTemperatureCompensation(Sensor_ts *sensor, uint8_t tl_magBF, uint8_t cpBF, uint8_t mtc);
bool tlx493d_gen_2_set1ByteReadMode(Sensor_ts *sensor, uint8_t prBF, uint8_t fpBF, uint8_t prdBF, uint8_t pr);

// TODO: cleanup
uint8_t tlx493d_gen_2_calculateFuseParity(Sensor_ts *sensor, uint8_t fpBF, uint8_t prdBF);
uint8_t tlx493d_gen_2_calculateBusParity(Sensor_ts *sensor, uint8_t to);
uint8_t tlx493d_gen_2_calculateConfigurationParity(Sensor_ts *sensor, uint8_t cpBF);
uint8_t tlx493d_gen_2_calculateConfigurationParityWakeup(Sensor_ts *sensor, uint8_t cpBF, uint8_t from, uint8_t to);

bool tlx493d_gen_2_hasValidFuseParity(Sensor_ts *sensor, uint8_t ffBF);
bool tlx493d_gen_2_hasValidBusParity(Sensor_ts *sensor, uint8_t pBF);
bool tlx493d_gen_2_hasValidConfigurationParity(Sensor_ts *sensor, uint8_t cpBF);

bool tlx493d_gen_2_hasValidData(Sensor_ts *sensor);
bool tlx493d_gen_2_hasValidTemperatureData(Sensor_ts *sensor);
bool tlx493d_gen_2_hasValidMagneticFieldData(Sensor_ts *sensor);

bool tlx493d_gen_2_isFunctional(Sensor_ts *sensor);

bool tlx493d_gen_2_hasValidTBit(Sensor_ts *sensor, uint8_t tBF) ;
bool tlx493d_gen_2_hasValidPD3Bit(Sensor_ts *sensor, uint8_t pd3BF);
bool tlx493d_gen_2_hasValidPD0Bit(Sensor_ts *sensor, uint8_t pd0BF);

bool tlx493d_gen_2_hasValidIICadr(Sensor_ts *sensor, uint8_t idBF, uint8_t iicAdrBF);
bool tlx493d_gen_2_hasWakeup(Sensor_ts *sensor, uint8_t typeBF);

uint8_t tlx493d_gen_2_getID(Sensor_ts *sensor, uint8_t idBF);
uint8_t tlx493d_gen_2_getFrameCounter(Sensor_ts *sensor, uint8_t frmBF) ;
uint8_t tlx493d_gen_2_getType(Sensor_ts *sensor, uint8_t typeBF);
uint8_t tlx493d_gen_2_getHWV(Sensor_ts *sensor, uint8_t hwvBF);


#endif // TLX493D_GEN_2_COMMON_H