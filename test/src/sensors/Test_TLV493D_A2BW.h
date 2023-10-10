// std includes
#include <string.h>

// test includes
#include "Test_includes.h"
#include "Test_sensors_common.h"
#include "Test_sensors_gen_2_common.h"


void TLV493D_A2BW_suiteSetUp(void);
void TLV493D_A2BW_suiteTearDown(void);


// variables used in the tests below that have to be accessed in the setup and tear down methods
// static Sensor_ts dut;


// define test group name
TEST_GROUP(TLV493D_A2BW);


// Setup method called before every individual test defined for this test group
TEST_SETUP(TLV493D_A2BW)
{
}


// Tear down method called before every individual test defined for this test group
TEST_TEAR_DOWN(TLV493D_A2BW)
{
}


// Define all relevant tests for the sensor device
TEST(TLV493D_A2BW, dummy)
{
    TEST_ASSERT( true == !false );
}


// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(TLV493D_A2BW)
{
    TLV493D_A2BW_suiteSetUp();

    RUN_TEST_CASE(TLV493D_A2BW, dummy);

    // run gen 2 common functions tests
    RUN_TEST_GROUP(SensorsCommon);
    RUN_TEST_GROUP(SensorsGen2Common);

    TLV493D_A2BW_suiteTearDown();
}
