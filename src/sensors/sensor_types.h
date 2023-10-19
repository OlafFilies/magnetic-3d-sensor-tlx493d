#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H


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
               TLE493D_A1B6_e = 0,
               TLE493D_A2B6_e,
               TLV493D_A2BW_e,
               TLE493D_P2B6_e,
               TLE493D_W2B6_e,
               TLV493D_W2BW_e,
               TLE493D_P3B6_e,
               TLE493D_P3I8_e } SupportedSensorTypes_te;


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
typedef void (*CalculateTemperatureFuncPtr)(Sensor_ts *, float *temp);
typedef bool (*GetTemperatureFuncPtr)(Sensor_ts *, float *temp);

// TODO: to be removed
typedef void (*CalculateFieldValuesFuncPtr)(Sensor_ts *, float *x, float *y, float *z);
typedef bool (*GetFieldValuesFuncPtr)(Sensor_ts *, float *x, float *y, float *z);

typedef bool (*GetSensorValuesFuncPtr)(Sensor_ts *, float *x, float *y, float *z, float *temp);

// typedef bool (*ResetFuncPtr)(Sensor_ts *);

typedef bool (*HasValidDataFuncPtr)(Sensor_ts *);
typedef bool (*IsFunctionalFuncPtr)(Sensor_ts *);

typedef bool (*SetDefaultConfigFuncPtr)(Sensor_ts *);
typedef bool (*ReadRegistersFuncPtr)(Sensor_ts *);

typedef bool (*EnableTemperatureFuncPtr)(Sensor_ts *);
typedef bool (*DisableTemperatureFuncPtr)(Sensor_ts *);

typedef bool (*EnableInterruptFuncPtr)(Sensor_ts *);
typedef bool (*DisableInterruptFuncPtr)(Sensor_ts *);

typedef bool (*SetPowerModeFuncPtr)(Sensor_ts *, uint8_t mode);
typedef bool (*SetIICAddressFuncPtr)(Sensor_ts *, uint8_t addr);

typedef bool (*TransferRegistersFuncPtr)(Sensor_ts *sensor, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len);


typedef struct CommonFunctions_ts {
    InitFuncPtr                     init;
    DeinitFuncPtr                   deinit;

// TODO: to be removed
    CalculateTemperatureFuncPtr     calculateTemperature;
    GetTemperatureFuncPtr           getTemperature;

// TODO: to be removed
    CalculateFieldValuesFuncPtr     calculateFieldValues;
    GetFieldValuesFuncPtr           getFieldValues;
    
    GetSensorValuesFuncPtr          getSensorValues;
    
    HasValidDataFuncPtr             hasValidData;
    IsFunctionalFuncPtr             isFunctional;

    // ResetFuncPtr                 reset;

    SetDefaultConfigFuncPtr         setDefaultConfig;
    ReadRegistersFuncPtr            readRegisters;

    EnableTemperatureFuncPtr        enableTemperature;
    DisableTemperatureFuncPtr       disableTemperature;

    EnableInterruptFuncPtr          enableInterrupt;
    DisableInterruptFuncPtr         disableInterrupt;

    SetPowerModeFuncPtr             setPowerMode;
    SetIICAddressFuncPtr            setIICAddress;

    // Functions used to refer to sensor specific functions by a common name. These functions are not part of the common user C/C++ interface.
    TransferRegistersFuncPtr        transfer;
} CommonFunctions_ts;


typedef struct CommonBitfields_ts {
    uint8_t CP;
    uint8_t FP;

    uint8_t ID;
    uint8_t P;
    uint8_t FF;
    uint8_t CF;
    uint8_t T;
    uint8_t PD3;
    uint8_t PD0;
    uint8_t FRM;
    uint8_t PRD;

    uint8_t TYPE;
    uint8_t HWV;

    uint8_t DT;
    uint8_t AM;
    uint8_t TRIG;
    uint8_t X2;
    uint8_t TL_MAG;

    uint8_t IICADR;
    uint8_t PR;
    uint8_t CA;
    uint8_t INT;
    uint8_t MODE;

    uint8_t BX_MSB;
    uint8_t BY_MSB;
    uint8_t BZ_MSB;
    uint8_t TEMP_MSB;
    uint8_t BX_LSB;
    uint8_t BY_LSB;
    uint8_t TEMP_LSB;
    uint8_t BZ_LSB;
    uint8_t TEMP2;
} CommonBitfields_ts;


typedef struct CommonRegisters_ts {
    uint8_t DIAG;
    uint8_t CONFIG;
    uint8_t MOD1;
    uint8_t MOD2;
    uint8_t VER;
} CommonRegisters_ts;


/*
  Structure to store all relevant infos for a particular sensor.
*/
typedef struct Sensor_ts {
    uint8_t                 *regMap;
    Register_ts             *regDef;
    CommonBitfields_ts       commonBitfields;
    CommonRegisters_ts       commonRegisters;
    CommonFunctions_ts      *functions;

    ComLibraryFunctions_ts  *comLibIF;
    ComLibraryParameters_ts  comLibIFParams;
    ComLibraryObject_ts      comLibObj;

    uint8_t                               regMapSize;
    SupportedSensorTypes_te               sensorType;
    SupportedComLibraryInterfaceTypes_te  comIFType;
} Sensor_ts;


#endif // SENSOR_TYPES_H