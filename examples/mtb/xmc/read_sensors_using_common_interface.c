// std includes
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// MTB includes
#include "cybsp.h"
#include "cy_utils.h"
#include "cy_retarget_io.h"

// XMC includes
#include "xmc_gpio.h"
#include "xmc_gpio.h"
#include "xmc_i2c.h"

// project c includes
#include "tlx493d_types.h"
#include "tlx493d_common.h"
#include "TLx493D_A2B6.h"


extern void frameworkReset(Sensor_ts *sensor);
extern void printRegMap(Sensor_ts *sensor);


int main(int argc, char *argv[]) {
    /* Initialize the device and board peripherals */
    cy_rslt_t result = cybsp_init();

    if (result != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
    }

    cy_retarget_io_init(CYBSP_DEBUG_UART_HW);

    Sensor_ts a2b6;

    init(&a2b6, TLx493D_A2B6_e);

    // XMC4700_Relax
    // initI2CComLibIF(&a2b6, XMC_I2C1_CH1, USIC1_C1_DX0_P3_15, USIC1_C1_DX1_P0_13, P3_15, P0_13);

    // XMC1100_Boot
    initComLibIF(&a2b6, XMC_I2C0_CH1, USIC0_C1_DX0_P2_10, USIC0_C1_DX1_P2_11, P2_10, P2_11);

    setDefaultConfig(&a2b6);
    printf("setDefaultConfig done.\n");
    printRegMap(&a2b6);
    // frameworkReset(&a2b6);
    assert(updateRegisterMap(&a2b6));
    printRegMap(&a2b6);
    setDefaultConfig(&a2b6);
    printf("setDefaultConfig done.\n");

    while( true ) {
        float x = 0, y = 0, z = 0;
        getFieldValues(&a2b6, &x, &y, &z);
        printf("x = %f   y = %f   z = %f\n", x, y, z);

        float temperature = 0;
        getTemperature(&a2b6, &temperature);
        printf("temperature = %f\n", temperature);

        // printf("sensorType : %s\n", getTypeAsString(a2b6.sensorType));
        // printf("\n\n");

        XMC_Delay(1000);
   }
}
