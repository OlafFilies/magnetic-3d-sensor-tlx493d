// std includes
#include <setjmp.h>
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
#include "xmc_i2c.h"

// Unity includes
#include "unity.h"

// project c includes
#include "sensor_types.h"
#include "sensors_common.h"
#include "TLE493D_A2B6.h"


Sensor_ts a2b6;


void initTests(void) {
   /* Initialize the device and board peripherals */
    cy_rslt_t result = cybsp_init();

    if (result != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
    }

    cy_retarget_io_init(CYBSP_DEBUG_UART_HW);

    init(&a2b6, TLE493D_A2B6_e, I2C_e);
    initI2CComLibIF(&a2b6, XMC_I2C1_CH1, USIC1_C1_DX0_P3_15, USIC1_C1_DX1_P0_13, P3_15, P0_13);
    setDefaultConfig(&a2b6);
}


void setUp(void) {
}



void tearDown(void) {
}


void resetTest(void) {
    tearDown();
    setUp();
}


void test_default_config(void) {
    TEST_ASSERT_EQUAL( true, updateRegisterMap(&a2b6) );
    TEST_ASSERT_EQUAL_HEX( 0x94, a2b6.regMap[0x11] );
}


void test_registers(void) {
}


int main(int argc, char *argv[]) {
    initTests();

    UnityBegin("test/xmc/ut_TLE493D_A2B6.c");
    
    RUN_TEST(test_default_config);
    RUN_TEST(test_registers);

    return UnityEnd();
}
