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

// Unity includes
#include "unity.h"

// project c includes
// sensor specicifc includes
#include "pal.h"
#include "sensors_common.h"
#include "TLE493D_A2B6.h"


extern void setup_i2c_relax_kit_4700_lite(void);


void setUp(void) {
}

void tearDown(void) {
    // clean stuff up here
}

void test_function_should_doBlahAndBlah(void) {
    //test stuff
    printf("test_function_should_doBlahAndBlah ...\n");
}

void test_function_should_doAlsoDoBlah(void) {
    //more test stuff
    printf("test_function_should_doAlsoDoBlah ...\n");
}


int main(int argc, char *argv[]) {
    // set stuff up here
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    cy_retarget_io_init(CYBSP_DEBUG_UART_HW);
    printf("main ...\n");

    setup_i2c_relax_kit_4700_lite();
 
    printf("testing ...\n");
    UNITY_BEGIN();
    
    RUN_TEST(test_function_should_doBlahAndBlah);
    RUN_TEST(test_function_should_doAlsoDoBlah);

    return UNITY_END();
}
