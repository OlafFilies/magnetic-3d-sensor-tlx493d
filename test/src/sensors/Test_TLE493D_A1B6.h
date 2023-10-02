// std includes
#include <string.h>

// test includes
#include "Test_includes.h"
#include "Test_sensors_common.h"
#include "Test_sensors_gen_1_common.h"


void TLE493D_A1B6_suiteSetUp(void);
void TLE493D_A1B6_suiteTearDown(void);


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
    TLE493D_A1B6_suiteSetUp();

    RUN_TEST_CASE(TLE493D_A1B6, dummy);

    // run gen 1 common functions tests
    RUN_TEST_GROUP(SensorsCommon);
    RUN_TEST_GROUP(SensorsGen1Common);

    TLE493D_A1B6_suiteTearDown();
}
