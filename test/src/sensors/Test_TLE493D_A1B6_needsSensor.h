// test includes
#include "Test_includes.h"


void TLE493D_A1B6_needsSensor_suiteSetup(void);
void TLE493D_A1B6_needsSensor_suiteTearDown(void);


// variables used in the tests below that have to be accessed in the setup and tear down methods
static Sensor_ts dut;


#include "Test_sensors_commonFunctions_needsSensor.h"


// define test group name
TEST_GROUP(TLE493D_A1B6_needsSensor);


// Setup method called before every individual test defined for this test group
TEST_SETUP(TLE493D_A1B6_needsSensor)
{
}


// Tear down method called before every individual test defined for this test group
TEST_TEAR_DOWN(TLE493D_A1B6_needsSensor)
{
}


// Define all relevant tests for the sensor device
TEST(TLE493D_A1B6_needsSensor, dummy)
{
    TEST_ASSERT( true == !false );
}


// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(TLE493D_A1B6_needsSensor)
{
    TLE493D_A1B6_needsSensor_suiteSetup();

    RUN_TEST_CASE(TLE493D_A1B6_needsSensor, dummy);
    
    // run common functions tests
    RUN_TEST_GROUP(SensorsCommonFunctions);

    TLE493D_A1B6_needsSensor_suiteTearDown();
}
