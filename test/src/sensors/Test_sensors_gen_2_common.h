// std includes
#include <string.h>

// test includes
#include "Test_includes.h"


// variables used in the tests below that have to be accessed in the setup and tear down methods
// static Sensor_ts dut;


// define test group name
TEST_GROUP(Sensors_gen_2_common);


// Setup method called before every individual test defined for this test group
TEST_SETUP(Sensors_gen_2_common)
{
}


// Tear down method called before every individual test defined for this test group
TEST_TEAR_DOWN(Sensors_gen_2_common)
{
}


// Define all relevant tests for the sensor device
TEST(Sensors_gen_2_common, dummy)
{
    TEST_ASSERT( true == !false );
}


// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(Sensors_gen_2_common)
{
    RUN_TEST_CASE(Sensors_gen_2_common, dummy);
}
