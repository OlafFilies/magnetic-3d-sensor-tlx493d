// std includes
#include <string.h>

// test includes
#include "Test_includes.h"


// variables used in the tests below that have to be accessed in the setup and tear down methods
// static Sensor_ts dut;


// define test group name
TEST_GROUP(TLE493D_A1B6);


// Setup method called before every individual test defined for this test group
TEST_SETUP(TLE493D_A1B6)
{
}


// Tear down method called before every individual test defined for this test group
TEST_TEAR_DOWN(TLE493D_A1B6)
{
}


// Define all relevant tests for the sensor device

TEST(TLE493D_A1B6, dummy)
{
    TEST_ASSERT( true == !false );
}


// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(TLE493D_A1B6)
{
    RUN_TEST_CASE(TLE493D_A1B6, dummy);
}
