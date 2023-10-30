// test includes
#include "Test_includes.h"
// #include "Test_utils.h"


void TLx493D_A2B6_needsSensor_suiteSetup(void);
void TLx493D_A2B6_needsSensor_suiteTearDown(void);


// variables used in the tests below that have to be accessed in the setup and tear down methods
static Sensor_ts dut;


#include "Test_tlx493d_commonFunctions_needsSensor.h"
#include "Test_tlx493d_common_needsSensor.h"
#include "Test_tlx493d_common.h"
#include "Test_tlx493d_gen_2_common_needsSensor.h"
#include "Test_tlx493d_gen_2_common.h"


// define test group name
TEST_GROUP(TLx493D_A2B6_needsSensor);


// Setup method called before every individual test defined for this test group
TEST_SETUP(TLx493D_A2B6_needsSensor)
{
    // for(auto c : "\nTEST_SETUP(TLx493D_A2B6_needsSensor) ...\n\n")
    //     putCharacter(c);

    // // 'main' initializes at startup, so either init everything or nothing at all, otherwise communication will be lost !
    // (void) TLx493D_A2B6_init(&dut);
    // initI2CComLibIF(&dut, Wire);
    // setDefaultConfig(&dut);
}


// Tear down method called before every individual test defined for this test group
TEST_TEAR_DOWN(TLx493D_A2B6_needsSensor)
{
    // for(auto c : "TEST_TEAR_DOWN(TLx493D_A2B6_needsSensor) ...\n\n")
    //     putCharacter(c);

    // // If deinitializing here make sure to reinit in 'TEST_SETUP' or communication will be lost !
    // (void) TLx493D_A2B6_deinit(&dut);
}


// Define all relevant tests for the sensor device

TEST(TLx493D_A2B6_needsSensor, readRegisters)
{
    TEST_ASSERT_EQUAL( true, tlx493d_readRegisters(&dut) );
//    TEST_ASSERT_EQUAL( true, readRegisters(&dut) );
}


TEST(TLx493D_A2B6_needsSensor, getTemperature)
{
    double temperature = 0.0;

    TEST_ASSERT_EQUAL( true, tlx493d_getTemperature(&dut, &temperature));
    TEST_ASSERT_FLOAT_WITHIN( 20.0, 25.0, temperature );
}


// Check if setDefaultConfig worked properly and data can be read and expected values are set.
TEST(TLx493D_A2B6_needsSensor, defaultConfig)
{
    TEST_ASSERT_EQUAL_HEX( 0x94, dut.regMap[0x11] );
}


TEST(TLx493D_A2B6_needsSensor, dummy)
{
    TEST_ASSERT( true == !false );
}


// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(TLx493D_A2B6_needsSensor)
{
    TLx493D_A2B6_needsSensor_suiteSetup();

    RUN_TEST_CASE(TLx493D_A2B6_needsSensor, readRegisters);
    RUN_TEST_CASE(TLx493D_A2B6_needsSensor, getTemperature);
    RUN_TEST_CASE(TLx493D_A2B6_needsSensor, defaultConfig);
    RUN_TEST_CASE(TLx493D_A2B6_needsSensor, dummy);

    
    // run common functions tests
    RUN_TEST_GROUP(SensorsCommonFunctions);

    // run gen 2 common functions tests
    RUN_TEST_GROUP(SensorsCommon);
    RUN_TEST_GROUP(SensorsGen2Common);
    RUN_TEST_GROUP(SensorsGen2Common_needsSensor);

    TLx493D_A2B6_needsSensor_suiteTearDown();
}

