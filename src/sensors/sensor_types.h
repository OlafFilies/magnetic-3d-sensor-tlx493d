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
               TLV493D_A1B6_e = 1,
               TLE493D_A2B6_e = 2,
               TLE493D_P2B6_e = 3,
               TLE493D_W2B6_e = 4 } SupportedSensorTypes_te;


/*
  List of all supported communictaion interfaces.
*/
typedef enum {
               I2C_e = 0,
               SPI_e } SupportedComLibraryInterfaceTypes_te;


/*
  List of supported register access modes.
*/
typedef enum { READ_MODE_e = 0, 
               WRITE_MODE_e } REG_ACESS_MODE_te;


/*
  Structure to store name, access mode, address, mask and offset of registers.
*/
typedef struct Register_ts {
    uint8_t            name;
    REG_ACESS_MODE_te  accessMode;
    uint8_t            address;
    uint8_t            mask;
    uint8_t            offset;
    // uint8_t           numBits;
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


// Functions common to all sensors
typedef bool (*InitFuncPtr)(Sensor_ts *, SupportedComLibraryInterfaceTypes_te comLibIF);
typedef bool (*DeinitFuncPtr)(Sensor_ts *);

typedef bool (*GetTemperatureFuncPtr)(Sensor_ts *, float *temp);
typedef bool (*UpdateGetTemperatureFuncPtr)(Sensor_ts *, float *temp);
typedef bool (*GetFieldValuesFuncPtr)(Sensor_ts *, float *x, float *y, float *z);
typedef bool (*UpdateGetFieldValuesFuncPtr)(Sensor_ts *, float *x, float *y, float *z);

typedef bool (*ResetFuncPtr)(Sensor_ts *);
typedef bool (*GetDiagnosisFuncPtr)(Sensor_ts *);
typedef bool (*CalculateParityFuncPtr)(Sensor_ts *);

typedef bool (*SetDefaultConfigFuncPtr)(Sensor_ts *);
typedef bool (*UpdateRegistersFuncPtr)(Sensor_ts *);


typedef struct CommonFunctions_ts {
    InitFuncPtr                  init;
    DeinitFuncPtr                deinit;

    GetTemperatureFuncPtr        getTemperature;
    UpdateGetTemperatureFuncPtr  updateGetTemperature;

    GetFieldValuesFuncPtr        getFieldValues;
    UpdateGetFieldValuesFuncPtr  updateGetFieldValues;

    ResetFuncPtr                 reset;
    GetDiagnosisFuncPtr          getDiagnosis;
    CalculateParityFuncPtr       calculateParity;

    SetDefaultConfigFuncPtr      setDefaultConfig;
    UpdateRegistersFuncPtr       updateRegisterMap;
} CommonFunctions_ts;


/*
  Structure to store all relevant infos for a particular sensor.
*/
typedef struct Sensor_ts {
    uint8_t                 *regMap;
    Register_ts             *regDef;
    ComLibraryFunctions_ts  *comLibIF;
    ComLibraryParameters_ts  comLibIFParams;
    
    #include "defineCommunicationLibraryObject.h"

    CommonFunctions_ts      *functions;

    uint8_t                               regMapSize;
    SupportedSensorTypes_te               sensorType;
    SupportedComLibraryInterfaceTypes_te  commIFType;
} Sensor_ts;


#endif // SENSOR_TYPES_H