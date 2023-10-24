// test includes
#include "Test_includes.h"


void TLE493D_W2B6_needsSensor_suiteSetup(void);
void TLE493D_W2B6_needsSensor_suiteTearDown(void);


// variables used in the tests below that have to be accessed in the setup and tear down methods
static Sensor_ts dut;


#include "Test_sensors_commonFunctions_needsSensor.h"
#include "Test_sensors_common_needsSensor.h"
#include "Test_sensors_common.h"
#include "Test_sensors_gen_2_common_needsSensor.h"
#include "Test_sensors_gen_2_common.h"


// define test group name
TEST_GROUP(TLE493D_W2B6_needsSensor);


// Setup method called before every individual test defined for this test group
TEST_SETUP(TLE493D_W2B6_needsSensor)
{
}


// Tear down method called before every individual test defined for this test group
TEST_TEAR_DOWN(TLE493D_W2B6_needsSensor)
{
}


// Define all relevant tests for the sensor device

TEST(TLE493D_W2B6_needsSensor, dummy)
{
    TEST_ASSERT( true == !false );
}

TEST(TLE493D_W2B6_needsSensor, concatBytes)
{
    int16_t result = 0;

    TEST_ASSERT(gen_2_readRegisters(&dut));
    TEST_ASSERT(gen_2_readRegisters(&dut));
    TEST_ASSERT(gen_2_readRegisters(&dut));

    
    
    gen_2_concatBytes(&dut, &dut.regDef[dut.commonBitfields.DT], &dut.regDef[dut.commonBitfields.MODE], &result);

    TEST_ASSERT_EQUAL(21, &dut.regDef[dut.commonBitfields.DT]);

    gen_2_concatBytes(&dut, &dut.regDef[dut.commonBitfields.PR], &dut.regDef[dut.commonBitfields.PR], &result);

    TEST_ASSERT_EQUAL(2, result);
}


// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(TLE493D_W2B6_needsSensor)
{
    TLE493D_W2B6_needsSensor_suiteSetup();

    RUN_TEST_CASE(TLE493D_W2B6_needsSensor, dummy);
        RUN_TEST_CASE(TLE493D_W2B6_needsSensor, concatBytes);

     
    // run common functions tests
    // RUN_TEST_GROUP(SensorsCommonFunctions);

    // run gen 2 common functions tests
    // RUN_TEST_GROUP(SensorsCommon);
    // RUN_TEST_GROUP(SensorsGen2Common);
    // RUN_TEST_GROUP(SensorsGen2Common_needsSensor);

    TLE493D_W2B6_needsSensor_suiteTearDown();
}
