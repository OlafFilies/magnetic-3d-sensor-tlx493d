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


int main(int argc, char *argv[]) {
    /* Initialize the device and board peripherals */
    cy_rslt_t result = cybsp_init();

    if (result != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
    }

    cy_retarget_io_init(CYBSP_DEBUG_UART_HW);

    Sensor_ts a2b6;

    TLx493D_A2B6_init(&a2b6);
    initComLibIF(&a2b6, XMC_I2C1_CH1, USIC1_C1_DX0_P3_15, USIC1_C1_DX1_P0_13, P3_15, P0_13);

    // reset(&a2b6);
    TLx493D_A2B6_setDefaultConfig(&a2b6);

    while( true ) {
        float x = 0, y = 0, z = 0;
        TLx493D_A2B6_getFieldValues(&a2b6, &x, &y, &z);
        printf("x = %f   y = %f   z = %f\n", x, y, z);

        float temperature = 0;
        TLx493D_A2B6_getTemperature(&a2b6, &temperature);
        printf("temperature = %f\n", temperature);

        // printf("sensorType : %s\n", getTypeAsString(a2b6.sensorType));
        // printf("\n\n");

        XMC_Delay(1000);
   }
}
