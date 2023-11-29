// std includes
#include <string.h>

// test includes
#include "Test_includes.h"


void TLx493D_A1B6_suiteSetUp(void);
void TLx493D_A1B6_suiteTearDown(void);

//defines


// variables used in the tests below that have to be accessed in the setup and tear down methods
static TLx493D_t dut;


// test includes that may require dut
#include "Test_tlx493d_common.h"


// static double  x, y, z, t;
static double  xl, xh, yl, yh, zl, zh;
static int16_t xl_i, xh_i, yl_i, yh_i, zl_i, zh_i;


// define test group name
TEST_GROUP(TLx493D_A1B6);
TEST_GROUP(TLx493D_A1B6_internal);


// Setup method called before every individual test defined for this test group
static TEST_SETUP(TLx493D_A1B6_internal)
{
    (void) TLx493D_A1B6_init(&dut);

    // certain compilers might not initialize the memory locations allocated by malloc()
    // so to ensure that the static (noSensor) unit tests pass, memset() is used to 
    // initialize to 0.
    memset(dut.regMap, 0, dut.regMapSize);

    // x = 0.0;
    // y = 0.0;
    // z = 0.0;
    // t = 0.0;

    // xl = 0.0;
    // xh = 0.0;
    // yl = 0.0;
    // yh = 0.0;
    // zl = 0.0;
    // zh = 0.0;

    // xl_i = 0;
    // xh_i = 0;
    // yl_i = 0;
    // yh_i = 0;
    // zl_i = 0;
    // zh_i = 0;
}


// Tear down method called before every individual test defined for this test group
static TEST_TEAR_DOWN(TLx493D_A1B6_internal)
{
    // dut.functions->deinit(&dut);
    (void) TLx493D_A1B6_deinit(&dut);
}


/**
 * Define tests for unsupported common functionality
 */
TEST_IFX(TLx493D_A1B6_internal, checkUnsupportedFunctionality)
{
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
TEST_IFX(TLx493D_A1B6_internal, checkSupportedFunctionality)
{
    // TEST_ASSERT( dut.functions->init(&dut) == true );
    // TEST_ASSERT( dut.functions->deinit(&dut) == true );
}


TEST_IFX(TLx493D_A1B6_internal, checkResetValues)
{
    // for(uint8_t i = 0; i < dut.regMapSize; ++i) {
    //     TEST_ASSERT( dut.regMap[i] == 0 );
    // }

    // dut.functions->setResetValues(&dut);

    // TEST_ASSERT( dut.regMap[0x10] == 0x00 ); // CONFIG
    // TEST_ASSERT( dut.regMap[0x11] == 0x00 ); // MOD1
    // TEST_ASSERT( dut.regMap[0x13] == 0x00 ); // MOD2
}


TEST_IFX(TLx493D_A1B6_internal, checkCalculateMagneticFieldAndTemperature)
{
    // double temperature = 0.0;
    // dut.functions->calculateTemperature(&dut, &temperature);
    // TEST_ASSERT_FLOAT_WITHIN( 1.0, -GEN_2_TEMP_OFFSET * GEN_2_TEMP_MULT + GEN_2_TEMP_REF, temperature );

    // double x = 0.0, y = 0.0, z = 0.0;
    // dut.functions->calculateMagneticField(&dut, &x, &y, &z);
    // TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, x );
    // TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, y );
    // TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, z );

    // temperature = 0.0;
    // x = 0.0;
    // y = 0.0;
    // z = 0.0;
    // dut.functions->calculateMagneticFieldAndTemperature(&dut, &x, &y, &z, &temperature);
    // TEST_ASSERT_FLOAT_WITHIN( 1.0, -GEN_2_TEMP_OFFSET * GEN_2_TEMP_MULT + GEN_2_TEMP_REF, temperature );
    // TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, x );
    // TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, y );
    // TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, z );
}


// Define all relevant tests for the sensor device

TEST_IFX(TLx493D_A1B6_internal, calculateTemperature)
{
    double temperature = 0.0;
    TLx493D_A1B6_calculateTemperature(&dut, &temperature);
    TEST_ASSERT_FLOAT_WITHIN( 0, (0.0 - GEN_1_TEMP_OFFSET) * GEN_1_TEMP_MULT, temperature );
}

TEST_IFX(TLx493D_A1B6_internal, calculateMagneticField)
{
    double x = 0.0, y = 0.0, z = 0.0;
    TLx493D_A1B6_calculateMagneticField(&dut, &x, &y, &z);
    TEST_ASSERT_FLOAT_WITHIN( 0, 0.0 * GEN_1_MAG_FIELD_MULT, x );
    TEST_ASSERT_FLOAT_WITHIN( 0, 0.0 * GEN_1_MAG_FIELD_MULT, y );
    TEST_ASSERT_FLOAT_WITHIN( 0, 0.0 * GEN_1_MAG_FIELD_MULT, z );
}

TEST_IFX(TLx493D_A1B6_internal, setter_BitFields)
{
    memset(dut.regMap,0,dut.regMapSize);

    // for gen1 setBitField is used to set values of WRITE registers (BitFields) only (due to a check for TLx493D_WRITE_MODE_e access)
    // so we test this function only for WRITE BitFields 
    for(uint8_t i=0; i < GEN_1_BITFIELDS_COUNT; i++){
        TLx493D_Register_t *bf = &dut.regDef[i];
        if (bf->accessMode == TLx493D_WRITE_MODE_e){
            TLx493D_A1B6_setBitfield(&dut, i, (pow(2, bf->numBits)-1)); //set all bits of bitfield as HIGH 
            TEST_ASSERT_EQUAL(pow(2,bf->numBits)-1, (dut.regMap[bf->address+GEN_1_WRITE_REGISTERS_OFFSET] & bf->mask) >> bf->offset); // check if it matches the same value, we set above
            // Note: The RegMap in gen1 for WRITE registers is shifted by a constant offset 
        }
    }
}

TEST_IFX(TLx493D_A1B6_internal, getter_BitFields)
{
    memset(dut.regMap,0,dut.regMapSize);

    // for gen1 returnBitField is used to get values of READ registers (BitFields) only (due to a check for TLx493D_READ_MODE_e access)
    // so we test this function only for READ BitFields 
    for(uint8_t i=0; i<GEN_1_BITFIELDS_COUNT; i++){
        TLx493D_Register_t *bf = &dut.regDef[i];
        if (bf->accessMode == TLx493D_READ_MODE_e){
            // first set the values (all bits as HIGH) of READ BitFields directly in the regMap
            dut.regMap[bf->address] = (dut.regMap[bf->address] & ~bf->mask) | ((uint8_t)(pow(2,bf->numBits)-1) << bf->offset);
            // then check against the same value as above
            TEST_ASSERT_EQUAL(pow(2,bf->numBits)-1, TLx493D_A1B6_returnBitfield(&dut, i));            
        }
    }
}


// Bundle all tests to be executed for this test group
static TEST_GROUP_RUNNER(TLx493D_A1B6_internal)
{
    RUN_TEST_CASE(TLx493D_A1B6_internal, checkUnsupportedFunctionality);
    RUN_TEST_CASE(TLx493D_A1B6_internal, checkSupportedFunctionality);

    RUN_TEST_CASE(TLx493D_A1B6_internal, checkResetValues);
    RUN_TEST_CASE(TLx493D_A1B6_internal, checkCalculateMagneticFieldAndTemperature);


    RUN_TEST_CASE(TLx493D_A1B6_internal, calculateTemperature);
    RUN_TEST_CASE(TLx493D_A1B6_internal, calculateMagneticField);
    RUN_TEST_CASE(TLx493D_A1B6_internal, setter_BitFields);
    RUN_TEST_CASE(TLx493D_A1B6_internal, getter_BitFields);
}


// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(TLx493D_A1B6)
{
    TLx493D_A1B6_suiteSetUp();

    RUN_TEST_GROUP(TLx493D_A1B6_internal);


#ifndef TEST_TLx493D_A1B6_NEEDS_SENSOR

    // run common functions tests
    RUN_TEST_GROUP(SensorsCommon);

#endif


    TLx493D_A1B6_suiteTearDown();
}
