// std includes
#include <string.h>

// test includes
#include "Test_includes.h"
// #include "Test_utils.h"


void TLx493D_A2B6_suiteSetUp(void);
void TLx493D_A2B6_suiteTearDown(void);


// variables used in the tests below that have to be accessed in the setup and tear down methods
static TLx493D_t dut;


// test includes that may require dut
#include "Test_tlx493d_common.h"
#include "Test_tlx493d_gen_2_common.h"


// define test group name
TEST_GROUP(TLx493D_A2B6);
TEST_GROUP(TLx493D_A2B6_internal);


// Setup method called before every individual test defined for this test group
static TEST_SETUP(TLx493D_A2B6_internal)
{
    // 'main' initializes at startup, so either init everything or nothing at all, otherwise communication will be lost !
    TLx493D_A2B6_init(&dut);

    memset(dut.regMap, 0, dut.regMapSize);
}


// Tear down method called before every individual test defined for this test group
static TEST_TEAR_DOWN(TLx493D_A2B6_internal)
{
    // If deinitializing here make sure to reinit in 'TEST_SETUP' or communication will be lost !
    dut.functions->deinit(&dut);
}


/**
 * Define tests for unsupported common functionality
 */
TEST_IFX(TLx493D_A2B6_internal, checkUnsupportedFunctionality)
{
    static double  xl, xh, yl, yh, zl, zh;
    static int16_t xl_i, xh_i, yl_i, yh_i, zl_i, zh_i;


    TEST_ASSERT( dut.functions->hasWakeUp(&dut) == false );
    TEST_ASSERT( dut.functions->isWakeUpEnabled(&dut) == false );
    TEST_ASSERT( dut.functions->enableWakeUpMode(&dut) == false );
    TEST_ASSERT( dut.functions->disableWakeUpMode(&dut) == false );

    TEST_ASSERT( dut.functions->setWakeUpThresholdsAsInteger(&dut, xh_i, xl_i, yh_i, yl_i, zh_i, zl_i) == false );
    TEST_ASSERT( dut.functions->setWakeUpThresholds(&dut, xh, xl, yh, yl, zh, zl) == false );


    TEST_ASSERT( dut.functions->softwareReset(&dut) == false );
}


/**
 * Define tests for supported common functionality.
 * Requires that the registers have been read once, in setDefaultConfig.
 */
TEST_IFX(TLx493D_A2B6_internal, checkSupportedFunctionality)
{
    TEST_ASSERT( dut.functions->init(&dut) == true );
    TEST_ASSERT( dut.functions->deinit(&dut) == true );
}


TEST_IFX(TLx493D_A2B6_internal, checkResetValues)
{
    for(uint8_t i = 0; i < dut.regMapSize; ++i) {
        TEST_ASSERT( dut.regMap[i] == 0 );
    }

    dut.functions->setResetValues(&dut);

    TEST_ASSERT( dut.regMap[0x10] == 0x00 ); // CONFIG
    TEST_ASSERT( dut.regMap[0x11] == 0x00 ); // MOD1
    TEST_ASSERT( dut.regMap[0x13] == 0x00 ); // MOD2
}


TEST_IFX(TLx493D_A2B6_internal, checkCalculateMagneticFieldAndTemperature)
{
    double temperature = 0.0;
    dut.functions->calculateTemperature(&dut, &temperature);
    TEST_ASSERT_FLOAT_WITHIN( 1.0, -GEN_2_TEMP_OFFSET * GEN_2_TEMP_RESOLUTION + GEN_2_TEMP_REF, temperature );

    double x = 0.0, y = 0.0, z = 0.0;
    dut.functions->calculateMagneticField(&dut, &x, &y, &z);
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, x );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, y );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, z );

    temperature = 0.0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
    dut.functions->calculateMagneticFieldAndTemperature(&dut, &x, &y, &z, &temperature);
    TEST_ASSERT_FLOAT_WITHIN( 1.0, -GEN_2_TEMP_OFFSET * GEN_2_TEMP_RESOLUTION + GEN_2_TEMP_REF, temperature );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, x );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, y );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, z );
}


// Bundle all tests to be executed for this test group
static TEST_GROUP_RUNNER(TLx493D_A2B6_internal)
{
    RUN_TEST_CASE(TLx493D_A2B6_internal, checkUnsupportedFunctionality);
    RUN_TEST_CASE(TLx493D_A2B6_internal, checkSupportedFunctionality);

    RUN_TEST_CASE(TLx493D_A2B6_internal, checkResetValues);
    RUN_TEST_CASE(TLx493D_A2B6_internal, checkCalculateMagneticFieldAndTemperature);
}


// Bundle all tests and internal groups to be executed for TLx493D_A2B6.
TEST_GROUP_RUNNER(TLx493D_A2B6)
{
    TLx493D_A2B6_suiteSetUp();

    RUN_TEST_GROUP(TLx493D_A2B6_internal);


#ifndef TEST_TLx493D_A2B6_NEEDS_SENSOR

    // run gen 2 common functions tests
    RUN_TEST_GROUP(SensorsCommon);
    RUN_TEST_GROUP(SensorsGen2Common);

#endif


    TLx493D_A2B6_suiteTearDown();
}
