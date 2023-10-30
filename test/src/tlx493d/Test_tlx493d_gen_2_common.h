#ifndef TEST_TLX493D_GEN_2_COMMON_H
#define TEST_TLX493D_GEN_2_COMMON_H


// std includes
#include <string.h>

// test includes
#include "Test_includes.h"


// Variables used in the tests below that have to be accessed in the setup and tear down methods.
// The "dut" variable is taken from the respective sensor that the common functions are applied to.


// define test group name
TEST_GROUP(SensorsGen2Common);


// Setup method called before every individual test defined for this test group
static TEST_SETUP(SensorsGen2Common)
{
}


// Tear down method called before every individual test defined for this test group
static TEST_TEAR_DOWN(SensorsGen2Common)
{
}


// Define all relevant tests for the sensor device
TEST_IFX(SensorsGen2Common, dummy)
{
    TEST_ASSERT( true == !false );
}


// Bundle all tests to be executed for this test group
static TEST_GROUP_RUNNER(SensorsGen2Common)
{
    RUN_TEST_CASE(SensorsGen2Common, dummy);
}


#endif // TEST_TLX493D_GEN_2_COMMON_H
