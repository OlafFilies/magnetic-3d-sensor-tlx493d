#ifndef TLX493D_TYPES_H
#define TLX493D_TYPES_H


// std includes
#include <stdbool.h>
#include <stdint.h>

// project c includes
#include "pal.h"


typedef struct Sensor_ts Sensor_ts;


/*
  List of all supported sensor devices.
*/
typedef enum {
               TLx493D_A1B6_e = 0,
               TLx493D_A2B6_e,
            //    TLx493D_A2BW_e,
               TLx493D_P2B6_e,
               TLx493D_W2B6_e,
               TLx493D_W2BW_e,
               TLx493D_P3B6_e,
               TLx493D_P3I8_e } SupportedSensorTypes_te;


/*
  List of all supported communication interfaces.
*/
typedef enum {
               I2C_e = 0,
               SPI_e,
               I2C_OR_SPI_e } SupportedComLibraryInterfaceTypes_te;


/*
  List of supported register access modes.
*/
typedef enum { READ_MODE_e = 0, 
               WRITE_MODE_e,
               READ_WRITE_MODE_e } REG_ACCESS_MODE_te;


/*
  Structure to store name, access mode, address, mask and offset of registers.
*/
typedef struct Register_ts {
    uint8_t             name;
    REG_ACCESS_MODE_te  accessMode;
    uint8_t             address;
    uint8_t             mask;
    uint8_t             offset;
    uint8_t             numBits;
} Register_ts;


typedef bool (*OneParamsFuncPtr)(Sensor_ts *sensor);
typedef bool (*TransferFuncPtr)(Sensor_ts *sensor, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len);


typedef struct SPIParameters_ts {
    uint8_t dummy;
} SPIParameters_ts;


typedef struct I2CParameters_ts {
    uint8_t address;
} I2CParameters_ts;


typedef struct ComLibraryFunctions_ts {
    union {
        OneParamsFuncPtr  spi_init;
        OneParamsFuncPtr  i2c_init;
    } init;

    union {
        OneParamsFuncPtr  spi_deinit;
        OneParamsFuncPtr  i2c_deinit;
    } deinit;

    union {
        TransferFuncPtr  spi_transfer;
        TransferFuncPtr  i2c_transfer;
    } transfer;
} ComLibraryFunctions_ts;


typedef union ComLibraryParameters_ts {
    SPIParameters_ts  spi_params;
    I2CParameters_ts  i2c_params;
} ComLibraryParameters_ts;


typedef union ComLibraryObject_ts {
    SPIObject_ts  *spi_obj;
    I2CObject_ts  *i2c_obj;
} ComLibraryObject_ts;


// Functions common to all sensors
typedef bool (*InitFuncPtr)(Sensor_ts *);
typedef bool (*DeinitFuncPtr)(Sensor_ts *);

// TODO: to be removed
typedef void (*CalculateTemperatureFuncPtr)(Sensor_ts *, double *);
// typedef bool (*GetTemperatureFuncPtr)(Sensor_ts *, double *);

// TODO: to be removed
typedef void (*CalculateMagneticFieldFuncPtr)(Sensor_ts *, double *, double *, double *);
// typedef bool (*GetMagneticFieldFuncPtr)(Sensor_ts *, double *, double *, double *);

typedef void (*CalculateMagneticFieldAndTemperatureFuncPtr)(Sensor_ts *, double *, double *, double *, double *);
// typedef bool (*GetMagneticFieldAndTemperatureFuncPtr)(Sensor_ts *, double *, double *, double *, double *);

typedef bool (*ReadRegistersFuncPtr)(Sensor_ts *);
typedef bool (*SetDefaultConfigFuncPtr)(Sensor_ts *);

typedef bool (*SetPowerModeFuncPtr)(Sensor_ts *, uint8_t);
typedef bool (*SetIICAddressFuncPtr)(Sensor_ts *, uint8_t);

typedef bool (*EnableTemperatureMeasurementFuncPtr)(Sensor_ts *);
typedef bool (*DisableTemperatureMeasurementFuncPtr)(Sensor_ts *);

typedef bool (*EnableInterruptFuncPtr)(Sensor_ts *);
typedef bool (*DisableInterruptFuncPtr)(Sensor_ts *);

typedef bool (*EnableAngularMeasurementFuncPtr)(Sensor_ts *);
typedef bool (*DisableAngularMeasurementFuncPtr)(Sensor_ts *);

typedef bool (*SetTriggerBitsFuncPtr)(Sensor_ts *, uint8_t);
typedef bool (*SetUpdateRateFuncPtr)(Sensor_ts *, uint8_t);

typedef bool (*IsWakeUpActiveFuncPtr)(Sensor_ts *sensor);
typedef bool (*EnableWakeUpModeFuncPtr)(Sensor_ts *sensor);
typedef bool (*DisableWakeUpModeFuncPtr)(Sensor_ts *sensor);

typedef bool (*SetLowerWakeUpThresholdXFuncPtr)(Sensor_ts *sensor, int16_t threshold);
typedef bool (*SetLowerWakeUpThresholdYFuncPtr)(Sensor_ts *sensor, int16_t threshold);
typedef bool (*SetLowerWakeUpThresholdZFuncPtr)(Sensor_ts *sensor, int16_t threshold);

typedef bool (*SetUpperWakeUpThresholdXFuncPtr)(Sensor_ts *sensor, int16_t threshold);
typedef bool (*SetUpperWakeUpThresholdYFuncPtr)(Sensor_ts *sensor, int16_t threshold);
typedef bool (*SetUpperWakeUpThresholdZFuncPtr)(Sensor_ts *sensor, int16_t threshold);

typedef bool (*SetWakeUpThresholdsFuncPtr)(Sensor_ts *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th);

typedef bool (*Enable1ByteReadModeFuncPtr)(Sensor_ts *);
typedef bool (*Disable1ByteReadModeFuncPtr)(Sensor_ts *);

typedef uint8_t (*CalculateFuseParityFuncPtr)(Sensor_ts *);
typedef uint8_t (*CalculateBusParityFuncPtr)(Sensor_ts *);
typedef uint8_t (*CalculateConfigParityFuncPtr)(Sensor_ts *);

typedef bool (*HasValidFuseParityFuncPtr)(Sensor_ts *);
typedef bool (*HasValidBusParityFuncPtr)(Sensor_ts *);
typedef bool (*HasValidConfigParityFuncPtr)(Sensor_ts *);

typedef bool (*HasValidDataFuncPtr)(Sensor_ts *);
typedef bool (*HasValidTemperatureDataFuncPtr)(Sensor_ts *);
typedef bool (*HasValidMagneticFieldDataFuncPtr)(Sensor_ts *);

typedef bool (*HasValidTBitFuncPtr)(Sensor_ts *);
typedef bool (*HasValidPD0BitFuncPtr)(Sensor_ts *);
typedef bool (*HasValidPD3BitFuncPtr)(Sensor_ts *);

typedef bool (*IsFunctionalFuncPtr)(Sensor_ts *);


// Functions used to refer to sensor specific functions by a common name. These functions are not part of the common user C/C++ interface.
typedef struct CommonFunctions_ts {
    InitFuncPtr                         init;
    DeinitFuncPtr                       deinit;

    ReadRegistersFuncPtr                readRegisters;
    SetDefaultConfigFuncPtr             setDefaultConfig;
    SetPowerModeFuncPtr                 setPowerMode;
    SetIICAddressFuncPtr                setIICAddress;

    CalculateTemperatureFuncPtr         calculateTemperature;
    // GetTemperatureFuncPtr               getTemperature;

    CalculateMagneticFieldFuncPtr       calculateMagneticField;
    // GetMagneticFieldFuncPtr             getMagneticField;
    
    CalculateMagneticFieldAndTemperatureFuncPtr  calculateMagneticFieldAndTemperature;
    // GetMagneticFieldAndTemperatureFuncPtr        getMagneticFieldAndTemperature;

    EnableTemperatureMeasurementFuncPtr   enableTemperatureMeasurement;
    DisableTemperatureMeasurementFuncPtr  disableTemperatureMeasurement;

    EnableAngularMeasurementFuncPtr     enableAngularMeasurement;
    DisableAngularMeasurementFuncPtr    disableAngularMeasurement;

    EnableInterruptFuncPtr              enableInterrupt;
    DisableInterruptFuncPtr             disableInterrupt;

    SetTriggerBitsFuncPtr               setTriggerBits;
    SetUpdateRateFuncPtr                setUpdateRate;

    IsWakeUpActiveFuncPtr               isWakeUpActive;
    EnableWakeUpModeFuncPtr             enableWakeUpMode;
    DisableWakeUpModeFuncPtr            disableWakeUpMode;

    SetLowerWakeUpThresholdXFuncPtr     setLowerWakeUpThresholdX;
    SetLowerWakeUpThresholdYFuncPtr     setLowerWakeUpThresholdY; 
    SetLowerWakeUpThresholdZFuncPtr     setLowerWakeUpThresholdZ;

    SetUpperWakeUpThresholdXFuncPtr     setUpperWakeUpThresholdX;
    SetUpperWakeUpThresholdYFuncPtr     setUpperWakeUpThresholdY; 
    SetUpperWakeUpThresholdZFuncPtr     setUpperWakeUpThresholdZ;

    SetWakeUpThresholdsFuncPtr          setWakeUpThresholds;                
    Enable1ByteReadModeFuncPtr          enable1ByteReadMode;
    Disable1ByteReadModeFuncPtr         disable1ByteReadMode;

    CalculateFuseParityFuncPtr          calculateFuseParity;
    CalculateBusParityFuncPtr           calculateBusParity;
    CalculateConfigParityFuncPtr        calculateConfigurationParity;
    
    HasValidFuseParityFuncPtr           hasValidFuseParity;
    HasValidBusParityFuncPtr            hasValidBusParity;
    HasValidConfigParityFuncPtr         hasValidConfigurationParity;

    HasValidDataFuncPtr                 hasValidData;
    HasValidTemperatureDataFuncPtr      hasValidTemperatureData;
    HasValidMagneticFieldDataFuncPtr    hasValidMagneticFieldData;

    HasValidTBitFuncPtr                 hasValidTBit;
    HasValidPD0BitFuncPtr               hasValidPD0Bit;
    HasValidPD3BitFuncPtr               hasValidPD3Bit;
    
    IsFunctionalFuncPtr                 isFunctional;
} CommonFunctions_ts;


// typedef struct CommonBitfields_ts {
//     uint8_t CP;
//     uint8_t FP;

//     uint8_t ID;
//     uint8_t P;
//     uint8_t FF;
//     uint8_t CF;
//     uint8_t T;
//     uint8_t PD3;
//     uint8_t PD0;
//     uint8_t FRM;
//     uint8_t PRD;

//     uint8_t TYPE;
//     uint8_t HWV;

//     uint8_t DT;
//     uint8_t AM;
//     uint8_t TRIG;
//     uint8_t X2;
//     uint8_t TL_MAG;

//     uint8_t IICADR;
//     uint8_t PR;
//     uint8_t CA;
//     uint8_t INT;
//     uint8_t MODE;

//     uint8_t BX_MSB;
//     uint8_t BY_MSB;
//     uint8_t BZ_MSB;
//     uint8_t TEMP_MSB;
//     uint8_t BX_LSB;
//     uint8_t BY_LSB;
//     uint8_t TEMP_LSB;
//     uint8_t BZ_LSB;
//     uint8_t TEMP2;

//     uint8_t CH;
//     uint8_t PD;
//     uint8_t FAST;
//     uint8_t LOW_POWER;
//     uint8_t LP;
//     uint8_t Temp_NEN;
//     uint8_t PT;
//     uint8_t R_RES_1;
//     uint8_t R_RES_2;
//     uint8_t R_RES_3;
//     uint8_t W_RES_0;
//     uint8_t W_RES_1;
//     uint8_t W_RES_2;
//     uint8_t W_RES_3;
// } CommonBitfields_ts;


// typedef struct CommonRegisters_ts {
//     uint8_t DIAG;
//     uint8_t CONFIG;
//     uint8_t MOD1;
//     uint8_t MOD2;
//     uint8_t VER;
// } CommonRegisters_ts;


/*
  Structure to store all relevant infos for a particular sensor.
*/
typedef struct Sensor_ts {
    uint8_t                 *regMap;
    Register_ts             *regDef;
    // CommonBitfields_ts       commonBitfields;
    // CommonRegisters_ts       commonRegisters;
    CommonFunctions_ts      *functions;

    ComLibraryFunctions_ts  *comLibIF;
    ComLibraryParameters_ts  comLibIFParams;
    ComLibraryObject_ts      comLibObj;

    uint8_t                               regMapSize;
    SupportedSensorTypes_te               sensorType;
    SupportedComLibraryInterfaceTypes_te  comIFType;
} Sensor_ts;


#endif // TLX493D_TYPES_H