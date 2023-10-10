// test includes
#include "Test_includes.h"


void TLV493D_A2BW_needsSensor_suiteSetup(void);
void TLV493D_A2BW_needsSensor_suiteTearDown(void);


// variables used in the tests below that have to be accessed in the setup and tear down methods
static Sensor_ts dut;


#include "Test_sensors_commonFunctions_needsSensor.h"
#include "Test_sensors_common_needsSensor.h"
#include "Test_sensors_common.h"
#include "Test_sensors_gen_2_common_needsSensor.h"
#include "Test_sensors_gen_2_common.h"


// define test group name
TEST_GROUP(TLV493D_A2BW_needsSensor);


// Setup method called before every individual test defined for this test group
TEST_SETUP(TLV493D_A2BW_needsSensor)
{
}


// Tear down method called before every individual test defined for this test group
TEST_TEAR_DOWN(TLV493D_A2BW_needsSensor)
{
}


// Define all relevant tests for the sensor device
TEST(TLV493D_A2BW_needsSensor, dummy)
{
    TEST_ASSERT( true == !false );
}


// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(TLV493D_A2BW_needsSensor)
{
    TLV493D_A2BW_needsSensor_suiteSetup();

    RUN_TEST_CASE(TLV493D_A2BW_needsSensor, dummy);
    
    // run common functions tests
    RUN_TEST_GROUP(SensorsCommonFunctions);

    // run gen 2 common functions tests
    RUN_TEST_GROUP(SensorsCommon);
    RUN_TEST_GROUP(SensorsGen2Common);
    RUN_TEST_GROUP(SensorsGen2Common_needsSensor);

    TLV493D_A2BW_needsSensor_suiteTearDown();
}
