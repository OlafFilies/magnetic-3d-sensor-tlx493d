#ifndef TLX493D_COMMUNICATION_INTERFACE_H
#define TLX493D_COMMUNICATION_INTERFACE_H


// std includes
#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus

extern "C" {

#endif


typedef struct TLx493D_t  TLx493D_t;


bool tlx493d_transfer(TLx493D_t *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen);
void tlx493d_setReadAddress(TLx493D_t *sensor, uint8_t address);


#ifdef __cplusplus

}

#endif


#ifdef __cplusplus

namespace ifx {
    namespace tlx493d {

#endif


void deinitCommunication(TLx493D_t *sensor);


#ifdef __cplusplus

    }
}

#endif


#endif // TLX493D_COMMUNICATION_INTERFACE_H
