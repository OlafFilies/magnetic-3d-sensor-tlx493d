#ifndef TEST_TLX493D_GEN_2_COMMON_NEEDS_SENSOR_H
#define TEST_TLX493D_GEN_2_COMMON_NEEDS_SENSOR_H


// test includes
#include "Test_includes.h"


// Variables used in the tests below that have to be accessed in the setup and tear down methods.
// The "dut" variable is taken from the respective sensor that the common functions are applied to.


// define test group name
TEST_GROUP(SensorsGen2Common_needsSensor);


// Setup method called before every individual test defined for this test group
static TEST_SETUP(SensorsGen2Common_needsSensor)
{
}


// Tear down method called before every individual test defined for this test group
static TEST_TEAR_DOWN(SensorsGen2Common_needsSensor)
{
}


// Define all relevant tests for the sensor device
TEST_IFX(SensorsGen2Common_needsSensor, dummy)
{
    TEST_ASSERT( true == !false );
}

TEST_IFX(SensorsGen2Common_needsSensor, concatBytes)
{
    int16_t result = 0;

    TEST_ASSERT(tlx493d_common_readRegisters(&dut));

    // tlx493d_common_concatBytes(&dut, 0x10, 0x11, &result);

    // TEST_ASSERT_EQUAL_INT16(0, result);

    // tlx493d_common_concatBytes(&dut, 0x13, 0x10, &result);

    // TEST_ASSERT_EQUAL_INT16(2, result);
}


// Bundle all tests to be executed for this test group
static TEST_GROUP_RUNNER(SensorsGen2Common_needsSensor)
{
    RUN_TEST_CASE(SensorsGen2Common_needsSensor, dummy);
    RUN_TEST_CASE(SensorsGen2Common_needsSensor, concatBytes);
}


#endif // TEST_TLX493D_GEN_2_COMMON_NEEDS_SENSOR_H
