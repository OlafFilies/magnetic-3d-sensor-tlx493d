#ifndef MTB_DEFINES_H
#define MTB_DEFINES_H


// std includes
#include <stdio.h>

// MTB includes

// XMC includes
#include "xmc_gpio.h"
#include "xmc_i2c.h"


#define log  printf


typedef struct TLx493D_I2CObject_t {
    XMC_USIC_CH_t    *channel;
    uint8_t           sourceSDA;
    uint8_t           sourceSCL;
    XMC_GPIO_PORT_t  *portSDA;
    uint8_t           pinSDA;
    XMC_GPIO_PORT_t  *portSCL;
    uint8_t           pinSCL;
} TLx493D_I2CObject_t;


typedef struct TLx493D_SPIObject_t  TLx493D_SPIObject_t;
typedef struct TLx493D_t            TLx493D_t;


extern bool initI2CComLibIF(TLx493D_t *sensor, XMC_USIC_CH_t *const channel,
                            const uint8_t sourceSDA, const uint8_t sourceSCL,
                            XMC_GPIO_PORT_t *const portSDA, const uint8_t pinSDA,
                            XMC_GPIO_PORT_t *const portSCL, const uint8_t pinSCL);


#endif // MTB_DEFINES_H
