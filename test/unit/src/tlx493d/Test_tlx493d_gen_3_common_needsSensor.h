#ifndef TEST_TLX493D_GEN_3_COMMON_NEEDS_SENSOR_H
#define TEST_TLX493D_GEN_3_COMMON_NEEDS_SENSOR_H


// test includes
#include "Test_includes.h"


// Variables used in the tests below that have to be accessed in the setup and tear down methods.
// The "dut" variable is taken from the respective sensor that the common functions are applied to.


// define test group name
TEST_GROUP(SensorsGen3Common_needsSensor);


// Setup method called before every individual test defined for this test group
static TEST_SETUP(SensorsGen3Common_needsSensor)
{
}


// Tear down method called before every individual test defined for this test group
static TEST_TEAR_DOWN(SensorsGen3Common_needsSensor)
{
}


// Define all relevant tests for the sensor device
TEST_IFX(SensorsGen3Common_needsSensor, dummy)
{
    TEST_ASSERT( true == !false );
}


// Bundle all tests to be executed for this test group
static TEST_GROUP_RUNNER(SensorsGen3Common_needsSensor)
{
    RUN_TEST_CASE(SensorsGen3Common_needsSensor, dummy);
}


#endif // TEST_TLX493D_GEN_3_COMMON_NEEDS_SENSOR_H
