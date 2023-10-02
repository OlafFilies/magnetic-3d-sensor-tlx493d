#ifndef TEST_SENSORS_GEN_1_COMMON
#define TEST_SENSORS_GEN_1_COMMON

// std includes
#include <string.h>

// test includes
#include "Test_includes.h"


// Variables used in the tests below that have to be accessed in the setup and tear down methods.
// The "dut" variable is taken from the respective sensor that the common functions are applied to.


// define test group name
TEST_GROUP(SensorsGen1Common);


// Setup method called before every individual test defined for this test group
static TEST_SETUP(SensorsGen1Common)
{
}


// Tear down method called before every individual test defined for this test group
static TEST_TEAR_DOWN(SensorsGen1Common)
{
}


// Define all relevant tests for the sensor device
TEST_IFX(SensorsGen1Common, dummy)
{
    TEST_ASSERT( true == !false );
}


// Bundle all tests to be executed for this test group
static TEST_GROUP_RUNNER(SensorsGen1Common)
{
    RUN_TEST_CASE(SensorsGen1Common, dummy);
}

#endif // TEST_SENSORS_GEN_1_COMMON
