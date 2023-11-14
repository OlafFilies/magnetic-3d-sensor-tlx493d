#ifndef TLX493D_PAL_H
#define TLX493D_PAL_H


// std includes
#include <stdbool.h>
#include <stdint.h>


#include "CommunicationInterface.h"


#ifdef __cplusplus

extern "C" {

#endif


typedef union TLx493D_ComLibraryParameters_t TLx493D_ComLibraryParameters_t;

typedef struct TLx493D_I2CObject_t TLx493D_I2CObject_t;
typedef struct TLx493D_SPIObject_t TLx493D_SPIObject_t;

typedef struct TLx493D_t   TLx493D_t;


// bool transfer(TLx493D_t *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen);
// void tlx493d_setI2CParameters(TLx493D_t *sensor, uint8_t addr);


#ifdef __cplusplus

}

#endif


#endif // TLX493D_PAL_H
