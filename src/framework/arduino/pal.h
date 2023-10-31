#ifndef TLX493D_PAL_H
#define TLX493D_PAL_H


// std includes
#include <stdbool.h>
#include <stdint.h>


typedef union TLx493D_ComLibraryParameters_ts TLx493D_ComLibraryParameters_ts;

typedef struct TLx493D_I2CObject_ts TLx493D_I2CObject_ts;
typedef struct TLx493D_SPIObject_ts TLx493D_SPIObject_ts;

typedef struct TLx493D_ts   TLx493D_ts;


#ifdef __cplusplus

extern "C" {

#endif


bool transfer(TLx493D_ts *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen);
void TLx493D_setI2CParameters(TLx493D_ts *sensor, uint8_t addr);


#ifdef __cplusplus

}

#endif


#endif // TLX493D_PAL_H
