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
#include "xmc_i2c.h"

// project c includes
#include "pal.h"
#include "sensors_common.h"
#include "TLE493D_A2B6.h"


extern void setup_i2c_relax_kit_4700_lite(void);


int main(int argc, char *argv[]) {
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    cy_retarget_io_init(CYBSP_DEBUG_UART_HW);

    setup_i2c_relax_kit_4700_lite();

    Sensor_ts a2b6;
    init(&a2b6, TLE493D_A2B6_e, I2C_e);
    setDefaultConfig(&a2b6);

    while( true ) {
        float x = 0, y = 0, z = 0;
        updateGetFieldValues(&a2b6, &x, &y, &z);
        printf("x = %f   y = %f   z = %f\n", x, y, z);

        float temperature = 0;
        updateGetTemperature(&a2b6, &temperature);
        printf("temperature = %f\n", temperature);

        printf("sensorType : %s\n", getTypeAsString(a2b6.sensorType));
        printf("\n\n");
        XMC_Delay(1000);
   }
}
