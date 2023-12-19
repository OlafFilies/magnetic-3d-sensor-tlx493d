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


// Setup method called before every individual test defined for this test group
static TEST_SETUP(TLx493D_P3B6_needsSensorInternal)
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
    t = 0.0;
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
    TEST_ASSERT( dut.functions->hasValidBusParity(&dut) == false );
    TEST_ASSERT( dut.functions->hasValidConfigurationParity(&dut) == false );
    TEST_ASSERT( dut.functions->hasValidTBit(&dut) == false );
 
    TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A3_e) == false );
    TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A2_e) == false );
    TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A1_e) == false );
    TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A0_e) == false );
}


/**
 * Define tests for supported common functionality.
 * Requires that the registers have been read once, in setDefaultConfig.
 */

TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkSupportedFunctionality)
{
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // printRegisters(&dut);
 
    // TEST_ASSERT( dut.functions->hasValidData(&dut) == true ); // fails sometimes

    TEST_ASSERT( dut.functions->isFunctional(&dut) == true );
    TEST_ASSERT( dut.functions->hasValidFuseParity(&dut) == true );
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkGetMagneticFieldAndTemperature)
{
    // TLx493D_FULL_RANGE_e
    TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_FULL_RANGE_e) == true );
    TEST_ASSERT( dut.functions->getTemperature(&dut, &t) == true );
    TEST_ASSERT_FLOAT_WITHIN( 20.0, 25.0, t );

    dut.functions->calculateMagneticField(&dut, &x, &y, &z);
    TEST_ASSERT_FLOAT_WITHIN( 2.0, 0.0, x );
    TEST_ASSERT_FLOAT_WITHIN( 2.0, 0.0, y );
    TEST_ASSERT_FLOAT_WITHIN( 2.0, 0.0, z );
    // TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, x );
    // TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, y );
    // TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, z );

    t = 0.0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
    dut.functions->calculateMagneticFieldAndTemperature(&dut, &x, &y, &z, &t);
    TEST_ASSERT_FLOAT_WITHIN( 20.0, 25.0, t );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, x );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, y );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, z );

print("x / y / z\n");
Serial.println(x);
Serial.println(y);
Serial.println(z);
Serial.println(t);
Serial.println();

    int16_t xr, yr, zr, tr;
    dut.functions->calculateRawMagneticFieldAndTemperature(&dut, &xr, &yr, &zr, &tr);
print("xr = %d   yr = %d   zr = %d   tr = %d\n", xr, yr, zr, tr);

    int16_t xr2, yr2, zr2;
    dut.functions->calculateRawMagneticFieldAtTemperature(&dut, tr, TLx493D_FULL_RANGE_e, x, y, z, &xr2, &yr2, &zr2);
print("xr2 = %d   yr2 = %d   zr2 = %d\n", xr2, yr2, zr2);

    TEST_ASSERT_INT16_WITHIN( 2, xr, xr2 );
    TEST_ASSERT_INT16_WITHIN( 2, yr, yr2 );
    TEST_ASSERT_INT16_WITHIN( 2, zr, zr2 );


    // TLx493D_EXTRA_SHORT_RANGE_e
    TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_EXTRA_SHORT_RANGE_e) == true );
    TEST_ASSERT( dut.functions->getTemperature(&dut, &t) == true );

    t = 0.0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
    dut.functions->calculateMagneticFieldAndTemperature(&dut, &x, &y, &z, &t);
print("x / y / z\n");
Serial.println(x);
Serial.println(y);
Serial.println(z);
Serial.println(t);
Serial.println();

    tr = 0;
    xr = 0;
    yr = 0;
    zr = 0;
    dut.functions->calculateRawMagneticFieldAndTemperature(&dut, &xr, &yr, &zr, &tr);
print("xr = %d   yr = %d   zr = %d   tr = %d\n", xr, yr, zr, tr);

    xr2 = 0;
    yr2 = 0;
    zr2 = 0;
    dut.functions->calculateRawMagneticFieldAtTemperature(&dut, tr, TLx493D_EXTRA_SHORT_RANGE_e, x, y, z, &xr2, &yr2, &zr2);
print("xr2 = %d   yr2 = %d   zr2 = %d\n", xr2, yr2, zr2);

    TEST_ASSERT_INT16_WITHIN( 2, xr, xr2 );
    TEST_ASSERT_INT16_WITHIN( 2, yr, yr2 );
    TEST_ASSERT_INT16_WITHIN( 2, zr, zr2 );


    // TLx493D_SHORT_RANGE_e
    TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_SHORT_RANGE_e) == true );
    TEST_ASSERT( dut.functions->getTemperature(&dut, &t) == true );

    t = 0.0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
    dut.functions->calculateMagneticFieldAndTemperature(&dut, &x, &y, &z, &t);
print("x / y / z\n");
Serial.println(x);
Serial.println(y);
Serial.println(z);
Serial.println(t);
Serial.println();

    tr = 0;
    xr = 0;
    yr = 0;
    zr = 0;
    dut.functions->calculateRawMagneticFieldAndTemperature(&dut, &xr, &yr, &zr, &tr);
print("xr = %d   yr = %d   zr = %d   tr = %d\n", xr, yr, zr, tr);

    xr2 = 0;
    yr2 = 0;
    zr2 = 0;
    dut.functions->calculateRawMagneticFieldAtTemperature(&dut, tr, TLx493D_SHORT_RANGE_e, x, y, z, &xr2, &yr2, &zr2);
print("xr2 = %d   yr2 = %d   zr2 = %d\n", xr2, yr2, zr2);

    TEST_ASSERT_INT16_WITHIN( 2, xr, xr2 );
    TEST_ASSERT_INT16_WITHIN( 2, yr, yr2 );
    TEST_ASSERT_INT16_WITHIN( 2, zr, zr2 );
    

    // back to TLx493D_FULL_RANGE_e
    TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_FULL_RANGE_e) == true );


    dut.functions->calculateRawMagneticFieldAtTemperature(&dut, tr, TLx493D_FULL_RANGE_e, 0.5, 0.5, 0.5, &xr, &yr, &zr);
print("xr = %d   yr = %d   zr = %d\n", xr, yr, zr);
    dut.functions->calculateRawMagneticFieldAtTemperature(&dut, tr, TLx493D_SHORT_RANGE_e, 0.5, 0.5, 0.5, &xr2, &yr2, &zr2);
print("xr2 = %d   yr2 = %d   zr2 = %d\n", xr2, yr2, zr2);

    TEST_ASSERT_INT16_WITHIN( 2, xr, xr2 );
    TEST_ASSERT_INT16_WITHIN( 2, yr, yr2 );
    TEST_ASSERT_INT16_WITHIN( 2, zr, zr2 );


    dut.functions->calculateRawMagneticFieldAtTemperature(&dut, tr, TLx493D_EXTRA_SHORT_RANGE_e, 0.5, 0.5, 0.5, &xr, &yr, &zr);
print("xr = %d   yr = %d   zr = %d\n", xr, yr, zr);

    TEST_ASSERT_INT16_WITHIN( 2, xr, xr2 );
    TEST_ASSERT_INT16_WITHIN( 2, yr, yr2 );
    TEST_ASSERT_INT16_WITHIN( 2, zr, zr2 );
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkBasicFunctionality)
{
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkConfigMeasurementFunctionality)
{
    // TLx493D_Register_t *channel = &dut.regDef[P3B6_CHANNEL_SEL_e];

    // switch to LPM
    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_LOW_POWER_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0x00, tlx493d_common_returnBitfield(&dut, P3B6_MODE_SEL_e) );



    // // switch to MCM
    // TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_MASTER_CONTROLLED_MODE_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT_EQUAL_HEX( 0x00, tlx493d_common_returnBitfield(&dut, P3B6_MODE_SEL_e) );


    // Unsupported
    TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxByBz_e) == false );


    // Supported
    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_VHall_Bias_e) == true );


    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_Spintest_e) == true );


    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_SAT_test_e) == true );


    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxTemp_e) == true );
    

//     TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxBy_e) == true );
//     TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
// print("TLx493D_BxBy_e\n");
// printRegisters(&dut);
//     TEST_ASSERT_EQUAL_HEX( 0b1101, tlx493d_common_returnBitfield(&dut, P3B6_CHANNEL_SEL_e) );

//     // TEST_ASSERT_EQUAL_HEX( 0x80, dut.regMap[0x04] ); // Bz MSBs
//     // TEST_ASSERT_EQUAL_HEX( 0x00, dut.regMap[0x05] & 0x3F ); // Bz LSBs
//     // TEST_ASSERT_EQUAL_HEX( 0x80, dut.regMap[0x06] ); // TEMP MSBs
//     // TEST_ASSERT_EQUAL_HEX( 0x00, dut.regMap[0x07] & 0x3F ); // TEMP LSBs


//     TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BzTemp_e) == true );
//     TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
// print("TLx493D_BzTemp_e\n");
// printRegisters(&dut);

//     TEST_ASSERT_EQUAL_HEX( 0b1110, tlx493d_common_returnBitfield(&dut, P3B6_CHANNEL_SEL_e) );

//     // TEST_ASSERT_EQUAL_HEX( 0x80, dut.regMap[0x00] ); // Bx MSBs
//     // TEST_ASSERT_EQUAL_HEX( 0x00, dut.regMap[0x01] & 0x3F ); // Bx LSBs
//     // TEST_ASSERT_EQUAL_HEX( 0x80, dut.regMap[0x02] ); // By MSBs
//     // TEST_ASSERT_EQUAL_HEX( 0x00, dut.regMap[0x03] & 0x3F ); // By LSBs


    //
    TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxByBzTemp_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
// print("TLx493D_BxByBzTemp_e\n");
// printRegisters(&dut);

    TEST_ASSERT_EQUAL_HEX( 0b0000, tlx493d_common_returnBitfield(&dut, P3B6_CHANNEL_SEL_e) );
    // printRegisters(&dut);
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkConfigTriggerFunctionality)
{
    // In low-power mode trigger selection is ignored !


    // MCM supports other modes, so enable MCM first
    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_MASTER_CONTROLLED_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true );
    TEST_ASSERT_EQUAL_HEX( 0x01, tlx493d_common_returnBitfield(&dut, P3B6_MODE_SEL_e) );

    // try triggers
    TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_ADC_ON_READ_AFTER_REG_05_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( (tlx493d_common_returnBitfield(&dut, P3B6_TRIGGER_SEL_e) == 0b10) || (tlx493d_common_returnBitfield(&dut, P3B6_TRIGGER_SEL_e) == 0b11) );

    //
    TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_ADC_ON_READ_BEFORE_FIRST_MSB_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0b01, tlx493d_common_returnBitfield(&dut, P3B6_TRIGGER_SEL_e) );

    //
    TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_NO_ADC_ON_READ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0b00, tlx493d_common_returnBitfield(&dut, P3B6_TRIGGER_SEL_e) );


    // switch back to LPM
    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_LOW_POWER_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0x00, tlx493d_common_returnBitfield(&dut, P3B6_MODE_SEL_e) );
    // printRegisters(&dut);
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkConfigSensitivityFunctionality)
{
    double sf;

    // Unsupported
    TEST_ASSERT( (tlx493d_common_returnBitfield(&dut, P3B6_SHORT_EN_e) & tlx493d_common_returnBitfield(&dut, P3B6_XTR_SHORT_EN_e)) != 1 );

    // supported
    TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_EXTRA_SHORT_RANGE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0x01, tlx493d_common_returnBitfield(&dut, P3B6_XTR_SHORT_EN_e) );
    TEST_ASSERT_EQUAL_HEX( 0x00, tlx493d_common_returnBitfield(&dut, P3B6_SHORT_EN_e) );

    sf = dut.functions->getSensitivityScaleFactor(&dut);
    TEST_ASSERT_EQUAL_FLOAT( 4.0, sf );


    TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_SHORT_RANGE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0x00, tlx493d_common_returnBitfield(&dut, P3B6_XTR_SHORT_EN_e) );
    TEST_ASSERT_EQUAL_HEX( 0x01, tlx493d_common_returnBitfield(&dut, P3B6_SHORT_EN_e) );

    sf = dut.functions->getSensitivityScaleFactor(&dut);
    TEST_ASSERT_EQUAL_FLOAT( 2.0, sf );
    

    TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_FULL_RANGE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0x00, tlx493d_common_returnBitfield(&dut, P3B6_XTR_SHORT_EN_e) );
    TEST_ASSERT_EQUAL_HEX( 0x00, tlx493d_common_returnBitfield(&dut, P3B6_SHORT_EN_e) );

    sf = dut.functions->getSensitivityScaleFactor(&dut);
    TEST_ASSERT_EQUAL_FLOAT( 1.0, sf );
    // printRegisters(&dut);
}


// Check if setDefaultConfig worked properly and data can be read and expected values are set.
TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkModeDefaultConfigFunctionality)
{
    // TEST_ASSERT( dut.functions->setDefaultConfig(&dut) == true);
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);

    TEST_ASSERT_EQUAL_HEX( 0x42, dut.regMap[P3B6_MOD1_REG_e] );
    TEST_ASSERT_EQUAL_HEX( 0x00, dut.regMap[P3B6_MOD2_REG_e] );
    // printRegisters(&dut);
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkModeCollisionAvoidanceFunctionality)
{
    TEST_ASSERT( dut.functions->disableCollisionAvoidance(&dut) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0x01, tlx493d_common_returnBitfield(&dut, P3B6_COLLISION_DIS_e) );

    TEST_ASSERT( dut.functions->enableCollisionAvoidance(&dut) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0x00, tlx493d_common_returnBitfield(&dut, P3B6_COLLISION_DIS_e) );
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkModeInterruptFunctionality)
{
    // TEST_ASSERT( dut.functions->enableInterrupt(&dut) == true );
    // TEST_ASSERT( (dut.regMap[P3B6_MOD1_REG_e] & 0x04) == 0x00 );

    TEST_ASSERT( dut.functions->disableInterrupt(&dut) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0x01, tlx493d_common_returnBitfield(&dut, P3B6_INT_DIS_e) );
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkModePowerModeFunctionality)
{
    // Unsupported
    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_FAST_MODE_e) == false );


    // Supported
    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_MASTER_CONTROLLED_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0x01, tlx493d_common_returnBitfield(&dut, P3B6_MODE_SEL_e) );


    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_LOW_POWER_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0x00, tlx493d_common_returnBitfield(&dut, P3B6_MODE_SEL_e) );
    // printRegisters(&dut);
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkModeUpdateRateFunctionality)
{
    // Valid only in LPM
    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_LOW_POWER_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0x00, tlx493d_common_returnBitfield(&dut, P3B6_MODE_SEL_e) );

    // Supported
    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_125_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0b01, tlx493d_common_returnBitfield(&dut, P3B6_F_UPDATE_SEL_e) );

    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_31_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0b10, tlx493d_common_returnBitfield(&dut, P3B6_F_UPDATE_SEL_e) );


    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_16_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0b11, tlx493d_common_returnBitfield(&dut, P3B6_F_UPDATE_SEL_e) );

    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_1000_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0b00, tlx493d_common_returnBitfield(&dut, P3B6_F_UPDATE_SEL_e) );


    // Unsupported
    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_97_HZ_e) == false );
    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_24_HZ_e) == false );
    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_12_HZ_e) == false );
    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_6_HZ_e) == false );

    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_3_HZ_e) == false );
    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_0_4_HZ_e) == false );
    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_0_05_HZ_e) == false );
    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_770_HZ_e) == false );

    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_SLOW_e) == false );
    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_FAST_e) == false );
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkWakeUpSettingsFunctionality)
{
    // Valid only in LPM
    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_LOW_POWER_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0x00, tlx493d_common_returnBitfield(&dut, P3B6_MODE_SEL_e) );


    TEST_ASSERT( dut.functions->disableWakeUpMode(&dut) == true );
    TEST_ASSERT( dut.functions->isWakeUpEnabled(&dut) == false );

    TEST_ASSERT( dut.functions->enableWakeUpMode(&dut) == true );
    TEST_ASSERT( dut.functions->isWakeUpEnabled(&dut) == true );

    TEST_ASSERT( dut.functions->disableWakeUpMode(&dut) == true );
}


TEST_IFX(TLx493D_P3B6_needsSensorInternal, checkWakeUpThresholdFunctionality)
{
    // pos. numbers
                                                              //   xlTh,         xhTh,        ylTh,        yhTh,        zlTh,        zhTh);
    TEST_ASSERT( dut.functions->setWakeUpThresholdsAsInteger(&dut, 0x0ABC >> 2, 0x00BC >> 2, 0x000C >> 2, 0x0FBC >> 2, 0x0F0C >> 2, 0x0F00 >> 2) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);

    // MSBs
    // threshold10Bits
    TEST_ASSERT_EQUAL_HEX( 0x0ABC >> 4, dut.regMap[0x0D] ); // XL
    TEST_ASSERT_EQUAL_HEX( 0x00BC >> 4, dut.regMap[0x0C] ); // XH
    TEST_ASSERT_EQUAL_HEX( 0x000C >> 4, dut.regMap[0x0F] ); // YL
    TEST_ASSERT_EQUAL_HEX( 0x0FBC >> 4, dut.regMap[0x0E] ); // YH
    TEST_ASSERT_EQUAL_HEX( 0x0F0C >> 4, dut.regMap[0x11] ); // ZL
    TEST_ASSERT_EQUAL_HEX( 0x0F00 >> 4, dut.regMap[0x10] ); // ZH

    // LSBs
    TEST_ASSERT_EQUAL_HEX( (0x0ABC >> 2) & 0x03, (dut.regMap[0x12] & 0x30) >> 4 ); // XL
    TEST_ASSERT_EQUAL_HEX( (0x00BC >> 2) & 0x03, (dut.regMap[0x12] & 0xC0) >> 6 ); // XH
    TEST_ASSERT_EQUAL_HEX( (0x000C >> 2) & 0x03, (dut.regMap[0x12] & 0x03) >> 0 ); // YL
    TEST_ASSERT_EQUAL_HEX( (0x0FBC >> 2) & 0x03, (dut.regMap[0x12] & 0x0C) >> 2 ); // YH
    TEST_ASSERT_EQUAL_HEX( (0x0F0C >> 2) & 0x03, (dut.regMap[0x13] & 0x03) >> 0 ); // ZL
    TEST_ASSERT_EQUAL_HEX( (0x0F00 >> 2) & 0x03, (dut.regMap[0x13] & 0x0C) >> 2 ); // ZH


    // // neg. numbers in hex format
    TEST_ASSERT( dut.functions->setWakeUpThresholdsAsInteger(&dut, ((int16_t) 0x8ABC) >> 2, ((int16_t) 0x80BC) >> 2, ((int16_t) 0x800C) >> 2,
                                                                   ((int16_t) 0x8FBC) >> 2, ((int16_t) 0x8F0C) >> 2, ((int16_t) 0x8F00) >> 2) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);

    // MSBs
    // threshold10Bits
    TEST_ASSERT_EQUAL_HEX( (((int16_t) 0x8ABC) >> 4) & 0xFF, dut.regMap[0x0D] ); // XL
    TEST_ASSERT_EQUAL_HEX( (((int16_t) 0x80BC) >> 4) & 0xFF, dut.regMap[0x0C] ); // XH
    TEST_ASSERT_EQUAL_HEX( (((int16_t) 0x800C) >> 4) & 0xFF, dut.regMap[0x0F] ); // YL
    TEST_ASSERT_EQUAL_HEX( (((int16_t) 0x8FBC) >> 4) & 0xFF, dut.regMap[0x0E] ); // YH
    TEST_ASSERT_EQUAL_HEX( (((int16_t) 0x8F0C) >> 4) & 0xFF, dut.regMap[0x11] ); // ZL
    TEST_ASSERT_EQUAL_HEX( (((int16_t) 0x8F00) >> 4) & 0xFF, dut.regMap[0x10] ); // ZH

    // LSBs
    TEST_ASSERT_EQUAL_HEX( (((int16_t) 0x8ABC) >> 2) & 0x03, (dut.regMap[0x12] & 0x30) >> 4 ); // XL
    TEST_ASSERT_EQUAL_HEX( (((int16_t) 0x80BC) >> 2) & 0x03, (dut.regMap[0x12] & 0xC0) >> 6 ); // XH
    TEST_ASSERT_EQUAL_HEX( (((int16_t) 0x800C) >> 2) & 0x03, (dut.regMap[0x12] & 0x03) >> 0 ); // YL
    TEST_ASSERT_EQUAL_HEX( (((int16_t) 0x8FBC) >> 2) & 0x03, (dut.regMap[0x12] & 0x0C) >> 2 ); // YH
    TEST_ASSERT_EQUAL_HEX( (((int16_t) 0x8F0C) >> 2) & 0x03, (dut.regMap[0x13] & 0x03) >> 0 ); // ZL
    TEST_ASSERT_EQUAL_HEX( (((int16_t) 0x8F00) >> 2) & 0x03, (dut.regMap[0x13] & 0x0C) >> 2 ); // ZH


    // neg. numbers in int format
    TEST_ASSERT( dut.functions->setWakeUpThresholdsAsInteger(&dut, -1 >> 2, -2 >> 2, -16 >> 2, -100 >> 2, -256 >> 2, -1024 >> 2) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);

    // MSBs
    // threshold10Bits
    TEST_ASSERT_EQUAL_HEX( (-1 >> 4) & 0xFF, dut.regMap[0x0D] ); // XL
    TEST_ASSERT_EQUAL_HEX( (-2 >> 4) & 0xFF, dut.regMap[0x0C] ); // XH
    TEST_ASSERT_EQUAL_HEX( (-16 >> 4) & 0xFF, dut.regMap[0x0F] ); // YL
    TEST_ASSERT_EQUAL_HEX( (-100 >> 4) & 0xFF, dut.regMap[0x0E] ); // YH
    TEST_ASSERT_EQUAL_HEX( (-256 >> 4) & 0xFF, dut.regMap[0x11] ); // ZL
    TEST_ASSERT_EQUAL_HEX( (-1024 >> 4) & 0xFF, dut.regMap[0x10] ); // ZH

    // LSBs
    TEST_ASSERT_EQUAL_HEX( (-1 >> 2) & 0x03, (dut.regMap[0x12] & 0x30) >> 4 ); // XL
    TEST_ASSERT_EQUAL_HEX( (-2 >> 2) & 0x03, (dut.regMap[0x12] & 0xC0) >> 6 ); // XH
    TEST_ASSERT_EQUAL_HEX( (-16 >> 2) & 0x03, (dut.regMap[0x12] & 0x03) >> 0 ); // YL
    TEST_ASSERT_EQUAL_HEX( (-100 >> 2) & 0x03, (dut.regMap[0x12] & 0x0C) >> 2 ); // YH
    TEST_ASSERT_EQUAL_HEX( (-256 >> 2) & 0x03, (dut.regMap[0x13] & 0x03) >> 0 ); // ZL
    TEST_ASSERT_EQUAL_HEX( (-1024 >> 2) & 0x03, (dut.regMap[0x13] & 0x0C) >> 2 ); // ZH
    // printRegisters(&dut);


    TEST_ASSERT( dut.functions->setWakeUpThresholds(&dut, 25.0, -5.0, 5.0, -5.0, 5.0, -5.0, 5.0) == true );

    int16_t th    = 0;
    uint8_t delta = 2;

    // X
    tlx493d_common_concatBytes(&dut, P3B6_WU_XL_MSBS_e, P3B6_WU_XL_LSBS_e, &th);
    TEST_ASSERT_INT16_WITHIN( delta, -153,  th ); // XL

    tlx493d_common_concatBytes(&dut, P3B6_WU_XH_MSBS_e, P3B6_WU_XH_LSBS_e, &th);
    TEST_ASSERT_INT16_WITHIN( delta, 154,  th ); // XH

    // Y
    tlx493d_common_concatBytes(&dut, P3B6_WU_YL_MSBS_e, P3B6_WU_YL_LSBS_e, &th);
    TEST_ASSERT_INT16_WITHIN( delta, -152,  th ); // YL

    tlx493d_common_concatBytes(&dut, P3B6_WU_YH_MSBS_e, P3B6_WU_YH_LSBS_e, &th);
    TEST_ASSERT_INT16_WITHIN( delta, 155,  th ); // YH

    // Z
    tlx493d_common_concatBytes(&dut, P3B6_WU_ZL_MSBS_e, P3B6_WU_ZL_LSBS_e, &th);
    TEST_ASSERT_INT16_WITHIN( delta, -151,  th ); // ZL

    tlx493d_common_concatBytes(&dut, P3B6_WU_ZH_MSBS_e, P3B6_WU_ZH_LSBS_e, &th);
    TEST_ASSERT_INT16_WITHIN( delta, 152,  th ); // ZH
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
