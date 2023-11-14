#ifndef TLX493D_COMMUNICATION_INTERFACE_H
#define TLX493D_COMMUNICATION_INTERFACE_H


// std includes
#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus

extern "C" {

#endif


typedef struct TLx493D_t  TLx493D_t;


bool transfer(TLx493D_t *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen);
void tlx493d_setI2CParameters(TLx493D_t *sensor, uint8_t addr);


#ifdef __cplusplus

}

#endif


#endif // TLX493D_COMMUNICATION_INTERFACE_H
