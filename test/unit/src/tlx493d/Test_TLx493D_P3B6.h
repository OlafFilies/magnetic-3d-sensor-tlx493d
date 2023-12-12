// std includes
#include <string.h>

// test includes
#include "Test_includes.h"


void TLx493D_P3B6_suiteSetUp(void);
void TLx493D_P3B6_suiteTearDown(void);


// variables used in the tests below that have to be accessed in the setup and tear down methods
static TLx493D_t dut;


// test includes that may require dut
#include "Test_tlx493d_common.h"
#include "Test_tlx493d_gen_3_common.h"


// define test group name
// define test group name
TEST_GROUP(TLx493D_P3B6);
TEST_GROUP(TLx493D_P3B6_internal);


// Setup method called before every individual test defined for this test group
static TEST_SETUP(TLx493D_P3B6_internal)
{
    TLx493D_P3B6_init(&dut);

    memset(dut.regMap, 0, dut.regMapSize);
}


// Tear down method called before every individual test defined for this test group
static TEST_TEAR_DOWN(TLx493D_P3B6_internal)
{
    dut.functions->deinit(&dut);
}


TEST_IFX(TLx493D_P3B6_internal, checkUnsupportedFunctionality)
{
}


/**
 * Define tests for supported common functionality.
 * Requires that the registers have been read once, in setDefaultConfig.
 */
TEST_IFX(TLx493D_P3B6_internal, checkSupportedFunctionality)
{
    TEST_ASSERT( dut.functions->init(&dut) == true );
    TEST_ASSERT( dut.functions->deinit(&dut) == true );

    TEST_ASSERT( dut.functions->hasWakeUp(&dut) == true );

    // TEST_ASSERT( dut.functions->softwareReset(&dut) == true );
}


TEST_IFX(TLx493D_P3B6_internal, checkResetValues)
{
    for(uint8_t i = 0; i < dut.regMapSize; ++i) {
        TEST_ASSERT( dut.regMap[i] == 0 );
    }

    dut.functions->setResetValues(&dut);

    TEST_ASSERT( dut.regMap[0x0A] == 0x62 ); // MOD1
    TEST_ASSERT( dut.regMap[0x0B] == 0x00 ); // MOD2

    // for wake-up parity calculation
    TEST_ASSERT( dut.regMap[0x0C] == 0x7F );
    TEST_ASSERT( dut.regMap[0x0D] == 0x80 );
    TEST_ASSERT( dut.regMap[0x0E] == 0x7F );
    TEST_ASSERT( dut.regMap[0x0F] == 0x80 );  
    TEST_ASSERT( dut.regMap[0x10] == 0x7F );
    TEST_ASSERT( dut.regMap[0x11] == 0x80 );
    TEST_ASSERT( dut.regMap[0x12] == 0xCC );
    TEST_ASSERT( dut.regMap[0x13] == 0x2C );
}


TEST_IFX(TLx493D_P3B6_internal, checkCalculateMagneticFieldAndTemperature)
{
    double temperature = 0.0;
    dut.functions->calculateTemperature(&dut, &temperature);
    TEST_ASSERT_FLOAT_WITHIN( 1.0, -GEN_3_TEMP_OFFSET / GEN_3_TEMP_SENSITIVITY + GEN_3_TEMP_REF, temperature );

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
    TEST_ASSERT_FLOAT_WITHIN( 1.0, -GEN_3_TEMP_OFFSET / GEN_3_TEMP_SENSITIVITY + GEN_3_TEMP_REF, temperature );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, x );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, y );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, z );
}


// Bundle all tests to be executed for this test group
static TEST_GROUP_RUNNER(TLx493D_P3B6_internal)
{
    RUN_TEST_CASE(TLx493D_P3B6_internal, checkUnsupportedFunctionality);
    RUN_TEST_CASE(TLx493D_P3B6_internal, checkSupportedFunctionality);

    RUN_TEST_CASE(TLx493D_P3B6_internal, checkResetValues);
    RUN_TEST_CASE(TLx493D_P3B6_internal, checkCalculateMagneticFieldAndTemperature);
}


// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(TLx493D_P3B6)
{
    TLx493D_P3B6_suiteSetUp();

    RUN_TEST_GROUP(TLx493D_P3B6_internal);


#ifndef TEST_TLx493D_P3B6_NEEDS_SENSOR

    // run gen 2 common functions tests
    RUN_TEST_GROUP(SensorsCommon);
    RUN_TEST_GROUP(SensorsGen2Common);

#endif


    TLx493D_P3B6_suiteTearDown();
}
