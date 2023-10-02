#ifndef TEST_SENSORS_COMMON
#define TEST_SENSORS_COMMON

// std includes
#include <string.h>

// test includes
#include "Test_includes.h"


// Variables used in the tests below that have to be accessed in the setup and tear down methods.
// The "dut" variable is taken from the respective sensor that the common functions are applied to.


// define test group name
TEST_GROUP(SensorsCommon);


// Setup method called before every individual test defined for this test group
static TEST_SETUP(SensorsCommon)
{
}


// Tear down method called before every individual test defined for this test group
static TEST_TEAR_DOWN(SensorsCommon)
{
}


// Define all relevant tests for the sensor device

TEST_IFX(SensorsCommon, dummy)
{
    TEST_ASSERT( true == !false );
}


// Bundle all tests to be executed for this test group
static TEST_GROUP_RUNNER(SensorsCommon)
{
    RUN_TEST_CASE(SensorsCommon, dummy);
}

#endif // TEST_SENSORS_COMMON
