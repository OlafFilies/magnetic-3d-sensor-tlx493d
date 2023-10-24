#ifndef PAL_H
#define PAL_H


// std includes
#include <stdbool.h>


typedef union ComLibraryParameters_ts ComLibraryParameters_ts;

typedef struct I2CObject_ts I2CObject_ts;
typedef struct SPIObject_ts SPIObject_ts;

typedef struct Sensor_ts    Sensor_ts;


#ifdef __cplusplus

extern "C" {

#endif


bool transfer(Sensor_ts *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen);

// TODO: replace by function pointers in comLibIF structure ??
void setI2CParameters(Sensor_ts *sensor, uint8_t addr);


#ifdef __cplusplus

}

#endif


#endif // PAL_H
