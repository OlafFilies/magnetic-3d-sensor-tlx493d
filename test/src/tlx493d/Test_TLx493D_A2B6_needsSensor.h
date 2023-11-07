// test includes
#include "Test_includes.h"
// #include "Test_utils.h"


void TLx493D_A2B6_needsSensor_suiteSetup(void);
void TLx493D_A2B6_needsSensor_suiteTearDown(void);


// variables used in the tests below that have to be accessed in the setup and tear down methods
static TLx493D_ts dut;


// test includes that may require dut
#include "Test_tlx493d_commonFunctions_needsSensor.h"
#include "Test_tlx493d_common_needsSensor.h"
#include "Test_tlx493d_common.h"
#include "Test_tlx493d_gen_2_common_needsSensor.h"
#include "Test_tlx493d_gen_2_common.h"


// define test group name
TEST_GROUP(TLx493D_A2B6_needsSensor);
TEST_GROUP(TLx493D_A2B6_needsSensorInternal);


static double  x, y, z, t;
static double  xl, xh, yl, yh, zl, zh;
static int16_t xl_i, xh_i, yl_i, yh_i, zl_i, zh_i;


// Setup method called before every individual test defined for this test group
static TEST_SETUP(TLx493D_A2B6_needsSensorInternal)
{
    // for(auto c : "\nTEST_SETUP(TLx493D_A2B6_needsSensor) ...\n\n")
    //     putCharacter(c);

    // print("\nTEST_SETUP(TLx493D_A2B6_needsSensor) ...\n\n");

    x = 0.0;
    y = 0.0;
    z = 0.0;
    t = 0.0;

    xl = 0.0;
    xh = 0.0;
    yl = 0.0;
    yh = 0.0;
    zl = 0.0;
    zh = 0.0;

    xl_i = 0;
    xh_i = 0;
    yl_i = 0;
    yh_i = 0;
    zl_i = 0;
    zh_i = 0;
}


// Tear down method called before every individual test defined for this test group
static TEST_TEAR_DOWN(TLx493D_A2B6_needsSensorInternal)
{
    // for(auto c : "\nTEST_TEAR_DOWN(TLx493D_A2B6_needsSensor) ...\n\n")
    //     putCharacter(c);

    // print("\nTEST_TEAR_DOWN(TLx493D_A2B6_needsSensor) ...\n\n");

}


/**
 * Define tests for unsupported common functionality.
 */
TEST_IFX(TLx493D_A2B6_needsSensorInternal, checkUnsupportedFunctionality)
{
}


/**
 * Define tests for supported common functionality.
 * Requires that the registers have been read once, in setDefaultConfig.
 */

TEST_IFX(TLx493D_A2B6_needsSensorInternal, checkSupportedFunctionality)
{
    TEST_ASSERT( dut.functions->hasValidData(&dut) == true ); // fails sometimes
    TEST_ASSERT( dut.functions->hasValidBusParity(&dut) == true ); // fails sometimes


    TEST_ASSERT( dut.functions->isFunctional(&dut) == true );

    TEST_ASSERT( dut.functions->hasValidFuseParity(&dut) == true );
    TEST_ASSERT( dut.functions->hasValidConfigurationParity(&dut) == true );

    TEST_ASSERT( dut.functions->hasValidTBit(&dut) == true );
}


// Check if setDefaultConfig worked properly and data can be read and expected values are set.
TEST_IFX(TLx493D_A2B6_needsSensorInternal, checkDefaultConfig)
{
    TEST_ASSERT_EQUAL_HEX( 0x00, dut.regMap[0x10] ); // all defaults
    TEST_ASSERT_EQUAL_HEX( 0x94, dut.regMap[0x11] ); // FP on, INT off, PR on
}


TEST_IFX(TLx493D_A2B6_needsSensorInternal, checkGetMagneticFieldAndTemperature)
{
    TEST_ASSERT( dut.functions->getTemperature(&dut, &t) == true );
    TEST_ASSERT_FLOAT_WITHIN( 20.0, 25.0, t );

    dut.functions->calculateMagneticField(&dut, &x, &y, &z);
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, x );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, y );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, z );

    t = 0.0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
    dut.functions->calculateMagneticFieldAndTemperature(&dut, &x, &y, &z, &t);
    TEST_ASSERT_FLOAT_WITHIN( 20.0, 25.0, t );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, x );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, y );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, z );
}


TEST_IFX(TLx493D_A2B6_needsSensorInternal, checkBasicFunctionality)
{
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true );
}


TEST_IFX(TLx493D_A2B6_needsSensorInternal, checkSetConfigFunctionality)
{
    // TLx493D_MeasureType_te mVals = ;
    // TEST_ASSERT( dut.functions->selectMeasureValues(&dut) == true );

    // TEST_ASSERT( dut.functions->setTrigger(&dut) == true );

    // TLx493D_SensitivityType_te sens = ;
    // TEST_ASSERT( dut.functions->setSensitivity(&dut) == true );
}


TEST_IFX(TLx493D_A2B6_needsSensorInternal, checkSetModeFunctionality)
{
    // TEST_ASSERT( dut.functions->setDefaultConfig(&dut) == true );

    // TLx493D_IICAddressType_te address = ;
    // TEST_ASSERT( dut.functions->setIICAddress(&dut, address) == true );

    // TEST_ASSERT( dut.functions->enableInterrupt(&dut) == true );
    // TEST_ASSERT( dut.functions->disableInterrupt(&dut) == true );

    // TEST_ASSERT( dut.functions->enableCollisionAvoidance(&dut) == true );
    // TEST_ASSERT( dut.functions->disableCollisionAvoidance(&dut) == true );

    // TLx493D_PowerModeType_te mode = ;
    // TEST_ASSERT( dut.functions->setPowerMode(&dut) == true );

    // TLx493D_UpdateRateType_te rate = ;
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut) == true );
}


static TEST_GROUP_RUNNER(TLx493D_A2B6_needsSensorInternal)
{
    RUN_TEST_CASE(TLx493D_A2B6_needsSensorInternal, checkUnsupportedFunctionality);
    RUN_TEST_CASE(TLx493D_A2B6_needsSensorInternal, checkSupportedFunctionality);

    RUN_TEST_CASE(TLx493D_A2B6_needsSensorInternal, checkDefaultConfig);
    RUN_TEST_CASE(TLx493D_A2B6_needsSensorInternal, checkBasicFunctionality);
    RUN_TEST_CASE(TLx493D_A2B6_needsSensorInternal, checkGetMagneticFieldAndTemperature);
    RUN_TEST_CASE(TLx493D_A2B6_needsSensorInternal, checkSetConfigFunctionality);
    RUN_TEST_CASE(TLx493D_A2B6_needsSensorInternal, checkSetModeFunctionality);
}


// Bundle all tests and internal groups to be executed for TLx493D_A2B6 with sensor attached.
TEST_GROUP_RUNNER(TLx493D_A2B6_needsSensor)
{
    TLx493D_A2B6_needsSensor_suiteSetup();

    // printRegisters(dut.regMap, dut.regMapSize);

    RUN_TEST_GROUP(TLx493D_A2B6_needsSensorInternal);
    
    // run common functions tests
    RUN_TEST_GROUP(SensorsCommonFunctions);

    // run gen 2 common functions tests
    RUN_TEST_GROUP(SensorsCommon);
    RUN_TEST_GROUP(SensorsGen2Common);
    RUN_TEST_GROUP(SensorsGen2Common_needsSensor);

    TLx493D_A2B6_needsSensor_suiteTearDown();
}

