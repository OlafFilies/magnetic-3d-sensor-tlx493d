// test includes
#include "Test_includes.h"


void TLx493D_P3B6_needsSensor_suiteSetup(void);
void TLx493D_P3B6_needsSensor_suiteTearDown(void);


// variables used in the tests below that have to be accessed in the setup and tear down methods
static TLx493D_t dut;


// test includes that may require dut
#include "Test_tlx493d_commonFunctions_needsSensor.h"
#include "Test_tlx493d_common_needsSensor.h"
#include "Test_tlx493d_common.h"
#include "Test_tlx493d_gen_3_common_needsSensor.h"
#include "Test_tlx493d_gen_3_common.h"


// define test group name
TEST_GROUP(TLx493D_P3B6_needsSensor);
TEST_GROUP(TLx493D_P3B6_needsSensorInternal);


static double  x, y, z, t;
// static double  xl, xh, yl, yh, zl, zh;
// static int16_t xl_i, xh_i, yl_i, yh_i, zl_i, zh_i;


// Setup method called before every individual test defined for this test group
static TEST_SETUP(TLx493D_P3B6_needsSensorInternal)
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
    t = 0.0;

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
static TEST_TEAR_DOWN(TLx493D_P3B6_needsSensorInternal)
{
}


// Define all relevant tests for the sensor device
/**
 * Define tests for unsupported common functionality.
 */
TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkUnsupportedFunctionality)
{
}


/**
 * Define tests for supported common functionality.
 * Requires that the registers have been read once, in setDefaultConfig.
 */

TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkSupportedFunctionality)
{
    TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // printRegisters(&dut);
 
    // TEST_ASSERT( dut.functions->hasValidData(&dut) == true ); // fails sometimes
    // TEST_ASSERT( dut.functions->hasValidBusParity(&dut) == true ); // fails sometimes

    TEST_ASSERT( dut.functions->isFunctional(&dut) == true );

    TEST_ASSERT( dut.functions->hasValidFuseParity(&dut) == true );
    TEST_ASSERT( dut.functions->hasValidConfigurationParity(&dut) == true );

    TEST_ASSERT( dut.functions->hasValidTBit(&dut) == true );
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkGetMagneticFieldAndTemperature)
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


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkBasicFunctionality)
{
    TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkConfigMeasurementFunctionality)
{
    // TLx493D_Register_t *dt       = &dut.regDef[P3B6_DT_e];
    // TLx493D_Register_t *am       = &dut.regDef[P3B6_AM_e];
    // TLx493D_Register_t *bzLSBS   = &dut.regDef[P3B6_BZ_LSBS_e];
    // TLx493D_Register_t *tempLSBS = &dut.regDef[P3B6_TEMP_LSBS_e];


    // // Unsupported
    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_VHall_Bias_e) == false );
    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_Spintest_e) == false );
    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_SAT_test_e) == false );
    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxTemp_e) == false );
    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BzTemp_e) == false );


    // // Supported
    // // TLx493D_BxBy_e
    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxBy_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);

    // TEST_ASSERT( (dut.regMap[P3B6_CONFIG_REG_e] & dt->mask) == dt->mask ); // DT
    // TEST_ASSERT( (dut.regMap[P3B6_CONFIG_REG_e] & am->mask) == am->mask ); // AM

    // TEST_ASSERT( dut.regMap[0x02] == 0x80 ); // Bz MSBS
    // TEST_ASSERT( dut.regMap[0x03] == 0x80 ); // TEMP MSBS
    // TEST_ASSERT( (dut.regMap[P3B6_TEMP2_REG_e] & (tempLSBS->mask | bzLSBS->mask)) == 0x00 ); // TEMP and Bz LSBS


    // // TLx493D_BxByBz_e
    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxByBz_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[P3B6_CONFIG_REG_e] & dt->mask) == dt->mask ); // DT
    // TEST_ASSERT( (dut.regMap[P3B6_CONFIG_REG_e] & am->mask) == 0x00 ); // AM is 0x00

    // TEST_ASSERT( dut.regMap[0x03] == 0x80 ); // TEMP MSB
    // TEST_ASSERT( (dut.regMap[P3B6_TEMP2_REG_e] & tempLSBS->mask) == 0x00 ); // TEMP


    // // TLx493D_BxByBzTemp_e
    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxByBzTemp_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[P3B6_CONFIG_REG_e] & 0x80) == 0x00 ); // DT
    // TEST_ASSERT( (dut.regMap[P3B6_CONFIG_REG_e] & 0x40) == 0x00 ); // AM
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkConfigTriggerFunctionality)
{
    // // switch to LPM
    // TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_LOW_POWER_MODE_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[P3B6_MOD1_REG_e] & 0x03) == 0x00 );

    // // Low-power mode only supports this trigger
    // TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_NO_ADC_ON_READ_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[P3B6_CONFIG_REG_e] & 0x30) == 0x00 );


    // // MCM supports other modes, so enable MCM first
    // TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_MASTER_CONTROLLED_MODE_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[P3B6_MOD1_REG_e] & 0x03) == 0x01 );

    // // try triggers
    // TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_ADC_ON_READ_AFTER_REG_05_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( ((dut.regMap[P3B6_CONFIG_REG_e] & 0x30) == 0x20) || ((dut.regMap[P3B6_CONFIG_REG_e] & 0x30) == 0x30) );

    // // Not to be used with our default config CA = 0, INT = 1 !
    // // TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_ADC_ON_READ_BEFORE_FIRST_MSB_e) == true );
    // // // while( tlx493d_common_readRegisters(&dut) == false ) ;
    // // TEST_ASSERT( (dut.regMap[P3B6_CONFIG_REG_e] & 0x30) == 0x10 );

    // TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_NO_ADC_ON_READ_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[P3B6_CONFIG_REG_e] & 0x30) == 0x00 );


    // // switch back to LPM
    // TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_LOW_POWER_MODE_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[P3B6_MOD1_REG_e] & 0x03) == 0x00 );
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkConfigSensitivityFunctionality)
{
    // double sf;

    // // unsupported
    // TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_EXTRA_SHORT_RANGE_e) == true );


    // // supported
    // TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_SHORT_RANGE_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[P3B6_CONFIG_REG_e] & 0x08) == 0x08 );

    // dut.functions->getSensitivityScaleFactor(&dut, &sf);
    // TEST_ASSERT_EQUAL_FLOAT( 2.0, sf );
    

    // TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_FULL_RANGE_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[P3B6_CONFIG_REG_e] & 0x08) == 0x00 );

    // dut.functions->getSensitivityScaleFactor(&dut, &sf);
    // TEST_ASSERT_EQUAL_FLOAT( 1.0, sf );
}


// Check if setDefaultConfig worked properly and data can be read and expected values are set.
TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkModeDefaultConfigFunctionality)
{
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // printRegisters(&dut);

    // TEST_ASSERT_EQUAL_HEX( dut.regMap[P3B6_CONFIG_REG_e], 0x00 );
    // TEST_ASSERT_EQUAL_HEX( dut.regMap[P3B6_MOD1_REG_e], 0x94 ); // PR on, CA on
    // TEST_ASSERT_EQUAL_HEX( dut.regMap[P3B6_MOD2_REG_e], 0x00 );
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkModeIICAddressFunctionality)
{
    // // printRegisters(&dut);
    // // print("addr : %x", dut.comLibIFParams.i2c_params.address << 1);

    // TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A3_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true );
    // TEST_ASSERT( (dut.regMap[P3B6_MOD1_REG_e] & 0x60) == 0x60 );
    // TEST_ASSERT( TLx493D_P3B6_hasValidIICadr(&dut) == true );

    // TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A2_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true );
    // TEST_ASSERT( (dut.regMap[P3B6_MOD1_REG_e] & 0x60) == 0x40 );
    // TEST_ASSERT( TLx493D_P3B6_hasValidIICadr(&dut) == true );

    // TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A1_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true );
    // TEST_ASSERT( (dut.regMap[P3B6_MOD1_REG_e] & 0x60) == 0x20 );
    // TEST_ASSERT( TLx493D_P3B6_hasValidIICadr(&dut) == true );

    // TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A0_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true );
    // TEST_ASSERT( (dut.regMap[P3B6_MOD1_REG_e] & 0x60) == 0x00 );
    TEST_ASSERT( TLx493D_P3B6_hasValidIICadr(&dut) == true );
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkModeCollisionAvoidanceFunctionality)
{
    TEST_ASSERT( dut.functions->disableCollisionAvoidance(&dut) == true );
    TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[P3B6_MOD1_REG_e] & 0x08) == 0x08 );

    TEST_ASSERT( dut.functions->enableCollisionAvoidance(&dut) == true );
    TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[P3B6_MOD1_REG_e] & 0x08) == 0x00 );
    TEST_ASSERT( dut.functions->enableCollisionAvoidance(&dut) == true );
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkModeInterruptFunctionality)
{
    // TEST_ASSERT( dut.functions->enableInterrupt(&dut) == true );
    // TEST_ASSERT( (dut.regMap[P3B6_MOD1_REG_e] & 0x04) == 0x00 );

    TEST_ASSERT( dut.functions->disableInterrupt(&dut) == true );
    TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[P3B6_MOD1_REG_e] & 0x04) == 0x04 );
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkModePowerModeFunctionality)
{
    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_FAST_MODE_e) == true );
    TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[P3B6_MOD1_REG_e] & 0x03) == 0x03 );

    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_MASTER_CONTROLLED_MODE_e) == true );
    TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[P3B6_MOD1_REG_e] & 0x03) == 0x01 );

    // forbidden
    TEST_ASSERT( (dut.regMap[P3B6_MOD1_REG_e] & 0x03) != 0x10 );

    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_LOW_POWER_MODE_e) == true );
    TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[P3B6_MOD1_REG_e] & 0x03) == 0x00 );
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkModeUpdateRateFunctionality)
{
    // // Supported
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_97_HZ_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[P3B6_MOD2_REG_e] & 0xE0) == 0x20 );


    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_24_HZ_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[P3B6_MOD2_REG_e] & 0xE0) == 0x40 );


    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_12_HZ_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[P3B6_MOD2_REG_e] & 0xE0) == 0x60 );


    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_6_HZ_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[P3B6_MOD2_REG_e] & 0xE0) == 0x80 );


    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_3_HZ_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[P3B6_MOD2_REG_e] & 0xE0) == 0xA0 );


    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_0_4_HZ_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[P3B6_MOD2_REG_e] & 0xE0) == 0xC0 );


    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_0_05_HZ_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[P3B6_MOD2_REG_e] & 0xE0) == 0xE0 );


    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_770_HZ_e) == true );
    // TEST_ASSERT( tlx493d_common_readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[P3B6_MOD2_REG_e] & 0xE0) == 0x00 );


    // // Unsupported
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_1000_HZ_e) == false );
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_125_HZ_e) == false );
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_31_HZ_e) == false );
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_16_HZ_e) == false );

    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_SLOW_e) == false );
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_FAST_e) == false );
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkWakeUpSettingsFunctionality)
{
    TEST_ASSERT( dut.functions->disableWakeUpMode(&dut) == true );
    TEST_ASSERT( dut.functions->isWakeUpEnabled(&dut) == false );

    TEST_ASSERT( dut.functions->enableWakeUpMode(&dut) == true );
    TEST_ASSERT( dut.functions->isWakeUpEnabled(&dut) == true );

    TEST_ASSERT( dut.functions->disableWakeUpMode(&dut) == true );
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkWakeUpThresholdFunctionality)
{
//     // pos. numbers
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, 0x0ABC) == true );
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, 0x00BC) == true );
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, 0x000C) == true );
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, 0x0FBC) == true );
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, 0x0F0C) == true );
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, 0x0F00) == true );


//    // neg. numbers
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, 0x8ABC) == true );
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, 0x80BC) == true );
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, 0x800C) == true );
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, 0x8FBC) == true );
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, 0x8F0C) == true );
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, 0x8F00) == true );

//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, -1) == true );
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, -2) == true );
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, -16) == true );
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, -100) == true );
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, -256) == true );
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, 0xFF0C) == true );
//     TEST_ASSERT( dut.functions->setLowerWakeUpThresholdX(&dut, 0xFF00) == true );
}


static TEST_GROUP_RUNNER(TLx493D_P3B6_needsSensorInternal)
{
    // First test default config applied in runner setup method 
    RUN_TEST_CASE(TLx493D_P3B6_needsSensorInternal, checkModeDefaultConfigFunctionality);

    RUN_TEST_CASE(TLx493D_P3B6_needsSensorInternal, checkUnsupportedFunctionality);
    RUN_TEST_CASE(TLx493D_P3B6_needsSensorInternal, checkSupportedFunctionality);

    RUN_TEST_CASE(TLx493D_P3B6_needsSensorInternal, checkBasicFunctionality);
    RUN_TEST_CASE(TLx493D_P3B6_needsSensorInternal, checkGetMagneticFieldAndTemperature);

    RUN_TEST_CASE(TLx493D_P3B6_needsSensorInternal, checkConfigMeasurementFunctionality);
    RUN_TEST_CASE(TLx493D_P3B6_needsSensorInternal, checkConfigTriggerFunctionality);
    RUN_TEST_CASE(TLx493D_P3B6_needsSensorInternal, checkConfigSensitivityFunctionality);

    // RUN_TEST_CASE(TLx493D_P3B6_needsSensorInternal, checkModeDefaultConfigFunctionality);
    RUN_TEST_CASE(TLx493D_P3B6_needsSensorInternal, checkModeIICAddressFunctionality);
    RUN_TEST_CASE(TLx493D_P3B6_needsSensorInternal, checkModeCollisionAvoidanceFunctionality);
    RUN_TEST_CASE(TLx493D_P3B6_needsSensorInternal, checkModeInterruptFunctionality);
    RUN_TEST_CASE(TLx493D_P3B6_needsSensorInternal, checkModePowerModeFunctionality);

    // MOD2
    RUN_TEST_CASE(TLx493D_P3B6_needsSensorInternal, checkModeUpdateRateFunctionality);

    // WakeUp functionality
    RUN_TEST_CASE(TLx493D_P3B6_needsSensorInternal, checkWakeUpSettingsFunctionality);
    RUN_TEST_CASE(TLx493D_P3B6_needsSensorInternal, checkWakeUpThresholdFunctionality);
}


// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(TLx493D_P3B6_needsSensor)
{
    TLx493D_P3B6_needsSensor_suiteSetup();

    RUN_TEST_GROUP(TLx493D_P3B6_needsSensorInternal);
     
    // run common functions tests
    RUN_TEST_GROUP(SensorsCommonFunctions);

    // run gen 2 common functions tests
    RUN_TEST_GROUP(SensorsCommon);
    RUN_TEST_GROUP(SensorsGen3Common);
    RUN_TEST_GROUP(SensorsGen3Common_needsSensor);

    TLx493D_P3B6_needsSensor_suiteTearDown();
}
