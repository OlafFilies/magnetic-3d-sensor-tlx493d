// test includes
#include "Test_includes.h"


void TLx493D_W2B6_needsSensor_suiteSetup(void);
void TLx493D_W2B6_needsSensor_suiteTearDown(void);


// variables used in the tests below that have to be accessed in the setup and tear down methods
static Sensor_ts dut;


#include "Test_tlx493d_commonFunctions_needsSensor.h"
#include "Test_tlx493d_common_needsSensor.h"
#include "Test_tlx493d_common.h"
#include "Test_tlx493d_gen_2_common_needsSensor.h"
#include "Test_tlx493d_gen_2_common.h"


// define test group name
TEST_GROUP(TLx493D_W2B6_needsSensor);


// Setup method called before every individual test defined for this test group
TEST_SETUP(TLx493D_W2B6_needsSensor)
{
}


// Tear down method called before every individual test defined for this test group
TEST_TEAR_DOWN(TLx493D_W2B6_needsSensor)
{
}


// Define all relevant tests for the sensor device

TEST(TLx493D_W2B6_needsSensor, dummy)
{
    TEST_ASSERT( true == !false );
}

TEST(TLx493D_W2B6_needsSensor, concatBytes)
{
    int16_t result = 0;

    TEST_ASSERT(tlx493d_readRegisters(&dut));
    TEST_ASSERT(tlx493d_readRegisters(&dut));
    TEST_ASSERT(tlx493d_readRegisters(&dut));
}


// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(TLx493D_W2B6_needsSensor)
{
    TLx493D_W2B6_needsSensor_suiteSetup();

    RUN_TEST_CASE(TLx493D_W2B6_needsSensor, dummy);
    RUN_TEST_CASE(TLx493D_W2B6_needsSensor, concatBytes);

     
    // run common functions tests
    // RUN_TEST_GROUP(SensorsCommonFunctions);

    // run gen 2 common functions tests
    // RUN_TEST_GROUP(SensorsCommon);
    // RUN_TEST_GROUP(SensorsGen2Common);
    // RUN_TEST_GROUP(SensorsGen2Common_needsSensor);

    TLx493D_W2B6_needsSensor_suiteTearDown();
}
