#ifndef MTB_DEFINES_H
#define MTB_DEFINES_H


// std includes
#include <stdio.h>

// MTB includes

// XMC includes
#include "xmc_gpio.h"
#include "xmc_i2c.h"


#define log  printf


typedef struct ComLibraryObject_ts {
    XMC_USIC_CH_t    *channel;
    uint8_t           sourceSDA;
    uint8_t           sourceSCL;
    XMC_GPIO_PORT_t  *portSDA;
    uint8_t           pinSDA;
    XMC_GPIO_PORT_t  *portSCL;
    uint8_t           pinSCL;
} ComLibraryObject_ts;


#endif // MTB_DEFINES_H
