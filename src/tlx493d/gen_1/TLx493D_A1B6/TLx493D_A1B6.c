// std includes
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"
#include "tlx493d_common_defines.h"
#include "tlx493d_common.h"

// common to same generation of sensors
// #include "tlx493d_gen_1_common_defines.h"
// #include "tlx493d_gen_1_common.h"

// sensor specicifc includes
#include "TLx493D_A1B6_defines.h"
#include "TLx493D_A1B6.h"

//register enums
typedef enum {
    TLx493D_A1B6_Temp_ENABLE_default,
    TLx493D_A1B6_Temp_DISABLE
} TLx493D_A1B6_Reg_Temp_NEN;

typedef enum {
    TLx493D_A1B6_ODD_PARITY,
    TLx493D_A1B6_EVEN_PARITY
} TLx493D_A1B6_Reg_PARITY;

typedef enum {
    TLx493D_A1B6_CONFIG_00_default,
    TLx493D_A1B6_CONFIG_01,
    TLx493D_A1B6_CONFIG_10,
    TLx493D_A1B6_CONFIG_11
} TLx493D_A1B6_Reg_IICADDR;

typedef enum {
    TLx493D_A1B6_INT_ENABLE_default,
    TLx493D_A1B6_INT_DISABLE
} TLx493D_A1B6_Reg_INT;

typedef enum {
    TLx493D_A1B6_FAST_MODE_DISABLE_default,
    TLx493D_A1B6_FAST_MODE_ENABLE
} TLx493D_A1B6_Reg_FAST_MODE_NEN;

typedef enum {
    TLx493D_A1B6_LOW_POWER_MODE_DISABLE_default,
    TLx493D_A1B6_LOW_POWER_MODE_ENABLE
} TLx493D_A1B6_Reg_LOW_POWER_MODE_NEN;

typedef enum {
    TLx493D_A1B6_PARITY_TEST_DISABLE,
    TLx493D_A1B6_PARITY_TEST_ENABLE_default
} TLx493D_A1B6_Reg_PARITY_TEST_NEN;

// framework functions
extern void setI2CParameters(Sensor_ts *sensor, uint8_t addr); 

/*
  Listing of all register names for this sensor.
  Used to index "TLx493D_A1B6_regDef" defined below, so index values must match !
*/
typedef enum {
               BX_MSB = 0,
               BY_MSB,
               BZ_MSB,
               TEMP_MSB,
               FRM,
               CH,
               BX_LSB,
               BY_LSB,
               T,
               FF,
               PD,
               BZ_LSB,
               TEMP_LSB,
               R_RES_1,
               R_RES_2,
               R_RES_3,
               W_RES_0,
               P,
               IICADR,
               W_RES_1,
               INT,
               FAST,
               LOW_POWER,
               W_RES_2,
               Temp_NEN,
               LP,
               PT,
               W_RES_3 } TLx493D_A1B6_registerNames_te;

Register_ts TLx493D_A1B6_regDef[] = {
    // Read registers
    {BX_MSB,        READ_MODE_e,    0x00, 0xFF, 0, 8},
    {BY_MSB,        READ_MODE_e,    0x01, 0xFF, 0, 8},
    {BZ_MSB,        READ_MODE_e,    0x02, 0xFF, 0, 8},
    {TEMP_MSB,      READ_MODE_e,    0x03, 0xF0, 4, 4},
    {FRM,           READ_MODE_e,    0x03, 0x0C, 2, 2},
    {CH,            READ_MODE_e,    0x03, 0x03, 0, 2},
    {BX_LSB,        READ_MODE_e,    0x04, 0xF0, 4, 4},
    {BY_LSB,        READ_MODE_e,    0x04, 0x0F, 0, 4},
    {T,             READ_MODE_e,    0x05, 0x40, 6, 1},
    {FF,            READ_MODE_e,    0x05, 0x20, 5, 1},
    {PD,            READ_MODE_e,    0x05, 0x10, 4, 1},
    {BZ_LSB,        READ_MODE_e,    0x05, 0x0F, 0, 4},
    {TEMP_LSB,      READ_MODE_e,    0x06, 0xFF, 0, 8},
    {R_RES_1,       READ_MODE_e,    0x07, 0x18, 3, 2},
    {R_RES_2,       READ_MODE_e,    0x08, 0xFF, 0, 8},
    {R_RES_3,       READ_MODE_e,    0x09, 0x1F, 0, 5},
    // Write Registers
    {W_RES_0,       WRITE_MODE_e,   0x00, 0xFF, 0, 8},
    {P,             WRITE_MODE_e,   0x01, 0x80, 7, 1},
    {IICADR,       WRITE_MODE_e,   0x01, 0x60, 5, 2},
    {W_RES_1,       WRITE_MODE_e,   0x01, 0x18, 3, 2},
    {INT,           WRITE_MODE_e,   0x01, 0x04, 2, 1},
    {FAST,          WRITE_MODE_e,   0x01, 0x02, 1, 1},
    {LOW_POWER,     WRITE_MODE_e,   0x01, 0x01, 0, 1},
    {W_RES_2,       WRITE_MODE_e,   0x02, 0xFF, 0, 8},
    {Temp_NEN,      WRITE_MODE_e,   0x03, 0x80, 7, 1},
    {LP,            WRITE_MODE_e,   0x03, 0x40, 6, 1},
    {PT,            WRITE_MODE_e,   0x03, 0x20, 5, 1},
    {W_RES_3,       WRITE_MODE_e,   0x03, 0x1F, 0, 5},
};

TLx493D_A1B6_PowerModeCombinations_t TLx493D_A1B6_PowerModeCombinations[] = {
    { 0, 0, 0, 1000 },		    // POWERDOWNMODE
	{ 1, 0, 0, 0    },			// FASTMODE
	{ 0, 1, 1, 10   },		    // LOWPOWERMODE
	{ 0, 1, 0, 100  },		    // ULTRALOWPOWERMODE
	{ 1, 1, 1, 10   }			// MASTERCONTROLLEDMODE
};

typedef enum { 
               MOD1_REG   = 0x01,
               MOD2_REG   = 0x03 } SpecialRegisters_te;           

CommonFunctions_ts TLx493D_A1B6_commonFunctions = {
                                .init                  = TLx493D_A1B6_init,
                                .deinit                = TLx493D_A1B6_deinit,

                                .calculateTemperature  = TLx493D_A1B6_calculateTemperature,
                                // .getTemperature        = TLx493D_A1B6_getTemperature,

                                .calculateMagneticField  = TLx493D_A1B6_calculateMagneticField,
                                // .getFieldValues        = TLx493D_A1B6_getMagneticField,

                                // .getMagneticFieldAndTemperature       = TLE493D_A2B6_getMagneticFieldAndTemperature,

                                // .reset                 = TLx493D_A1B6_reset,

                                .hasValidData          = TLx493D_A1B6_hasValidData,
                                .isFunctional          = TLx493D_A1B6_isFunctional,

                                .setDefaultConfig      = TLx493D_A1B6_setDefaultConfig,
                                .readRegisters         = TLx493D_A1B6_readRegisters,

                                .enableTemperatureMeasurement     = TLx493D_A1B6_enableTemperatureMeasurement,
                                .disableTemperatureMeasurement    = TLx493D_A1B6_disableTemperatureMeasurement, //TODO: rebase common functions as per new common functions struct 

                                .setIICAddress         = TLx493D_A1B6_setIICAddress,

                                // .transfer              = TLx493D_A1B6_transferRegisterMap,
                              };


bool TLx493D_A1B6_init(Sensor_ts *sensor) {
    sensor->regMap            = (uint8_t *) malloc(sizeof(uint8_t) * GEN_1_REG_MAP_SIZE);
    sensor->regDef            = TLx493D_A1B6_regDef;
    // sensor->commonBitfields   = (CommonBitfields_ts) {  .BX_MSB = BX_MSB, .BY_MSB = BY_MSB, 
    //                                                     .BZ_MSB = BZ_MSB, .TEMP_MSB = TEMP_MSB,
    //                                                     .BX_LSB = BX_LSB, .BY_LSB = BY_LSB, 
    //                                                     .T = T, .TEMP_LSB = TEMP_LSB, 
    //                                                     .BZ_LSB = BZ_LSB, .P = P, .FRM = FRM, 
    //                                                     .FF = FF, .CH = CH, .PD = PD, .IICADR = IICADR, 
    //                                                     .INT = INT, .FAST = FAST, .LOW_POWER = LOW_POWER, 
    //                                                     .LP = LP, .PT = PT, .Temp_NEN = Temp_NEN, 
    //                                                     .PT = PT, .R_RES_1 = R_RES_1, 
    //                                                     .R_RES_2 = R_RES_2, .R_RES_3 = R_RES_3, 
    //                                                     .W_RES_0 = W_RES_0, .W_RES_1 = W_RES_1, 
    //                                                     .W_RES_2 = W_RES_2, .W_RES_3 = W_RES_3
    //                                                  };
    // sensor->commonRegisters   = (CommonRegisters_ts) { .MOD1 = MOD1_REG, .MOD2 = MOD2_REG };                                                     
    sensor->functions         = &TLx493D_A1B6_commonFunctions;
    sensor->regMapSize        = GEN_1_REG_MAP_SIZE;
    sensor->sensorType        = TLx493D_A1B6_e;
    sensor->comIFType         = I2C_e;
    sensor->comLibIF          = NULL;
    sensor->comLibObj.i2c_obj = NULL;

    memset(sensor->regMap, 0, sensor->regMapSize);

    setI2CParameters(sensor, GEN_1_STD_IIC_ADDR);

    return true;
}

bool TLx493D_A1B6_deinit(Sensor_ts *sensor) {
    free(sensor->regMap);
    free(sensor->comLibObj.i2c_obj);

    sensor->regMap            = NULL;
    sensor->comLibObj.i2c_obj = NULL;

    return true;
}

void TLx493D_A1B6_setReservedRegisterValues(Sensor_ts *sensor) {
    TLx493D_A1B6_setBitfield(sensor, W_RES_1, TLx493D_A1B6_returnBitfield(sensor, R_RES_1));
    TLx493D_A1B6_setBitfield(sensor, W_RES_2, TLx493D_A1B6_returnBitfield(sensor, R_RES_2));
    TLx493D_A1B6_setBitfield(sensor, W_RES_3, TLx493D_A1B6_returnBitfield(sensor, R_RES_3));
}

// note: make sure that the init function is called at reset to make sure the write default values are in sync.

bool TLx493D_A1B6_setDefaultConfig(Sensor_ts *sensor) {
    
    bool ret = true;
    // read READ register values
    ret = TLx493D_A1B6_readRegisters(sensor);
    
    // set WRITE register values to 0x00
    memset(sensor->regMap+TLx493D_A1B6_WRITE_REGISTERS_OFFSET,0,TLx493D_A1B6_WRITE_REGISTERS_MAX_COUNT);

    // todo: reset sensor here

    // set WRITE reserved register values to READ reserved register values
    TLx493D_A1B6_setReservedRegisterValues(sensor);

    // enable parity test and write to registers 
    ret &= TLx493D_A1B6_enableParityTest(sensor);

    // set to MASTERCONTROLLEDMODE to start measurement
    TLx493D_A1B6_setPowerMode(sensor, MASTERCONTROLLEDMODE);

    // calculate parity
    TLx493D_A1B6_calculateParity(sensor);

    // write out register map to registers
    ret &= TLx493D_A1B6_transferWriteRegisters(sensor);

    // update register map
    ret &= TLx493D_A1B6_readRegisters(sensor);

    return ret;  
}

bool TLx493D_A1B6_disableTemperatureMeasurement(Sensor_ts *sensor) {
    
    TLx493D_A1B6_setBitfield(sensor, Temp_NEN, TLx493D_A1B6_Temp_DISABLE);
    TLx493D_A1B6_calculateParity(sensor);
    bool ret = TLx493D_A1B6_transferWriteRegisters(sensor);
    
    return ret;                                                           
}

bool TLx493D_A1B6_enableTemperatureMeasurement(Sensor_ts *sensor) {

    TLx493D_A1B6_setBitfield(sensor, Temp_NEN, TLx493D_A1B6_Temp_ENABLE_default);
    TLx493D_A1B6_calculateParity(sensor);
    bool ret = TLx493D_A1B6_transferWriteRegisters(sensor);

    return ret; 
}

bool TLx493D_A1B6_transferWriteRegisters(Sensor_ts *sensor) {
    
    bool retn = transfer(sensor, sensor->regMap + TLx493D_A1B6_WRITE_REGISTERS_OFFSET, TLx493D_A1B6_WRITE_REGISTERS_MAX_COUNT, NULL, 0);
    // bool retn = sensor->comLibIF->transfer.i2c_transfer(sensor, sensor->regMap + TLx493D_A1B6_WRITE_REGISTERS_OFFSET,
    //                                                     TLx493D_A1B6_WRITE_REGISTERS_MAX_COUNT, NULL, 0);
    
    return retn;
}

// parity is calculated for all the WRITE register, including the parity bit
void TLx493D_A1B6_calculateParity(Sensor_ts *sensor) {
    uint8_t result = 0x00;

    //set parity as EVEN first
    TLx493D_A1B6_setBitfield(sensor, P, TLx493D_A1B6_EVEN_PARITY); 

    // calculate bitwise XOR for all WRITE registers
    for (uint8_t addr = 0x00; addr<TLx493D_A1B6_WRITE_REGISTERS_MAX_COUNT; addr++) {
        result ^= sensor->regMap[addr+TLx493D_A1B6_WRITE_REGISTERS_OFFSET];
    }

    // then set calculated parity
    TLx493D_A1B6_setBitfield(sensor, P, tlx493d_calculateParity(result));
}

void TLx493D_A1B6_calculateTemperature(Sensor_ts *sensor, double *temp) {
    int16_t value = 0;
    tlx493d_concatBytes(sensor, TEMP_MSB, TEMP_LSB, &value);
    
    *temp = ((double)value - GEN_1_TEMP_OFFSET) * GEN_1_TEMP_MULT;
}

bool TLx493D_A1B6_getTemperature(Sensor_ts *sensor, double *temp) {
    if( TLx493D_A1B6_readRegisters(sensor) ) {
        TLx493D_A1B6_calculateTemperature(sensor, temp);
        return true;
    }
    return false;
}

bool TLx493D_A1B6_enableParityTest(Sensor_ts *sensor) {
    TLx493D_A1B6_setBitfield(sensor, PT, TLx493D_A1B6_PARITY_TEST_ENABLE_default);
    TLx493D_A1B6_calculateParity(sensor);
    return TLx493D_A1B6_transferWriteRegisters(sensor);
}

bool TLx493D_A1B6_disableParityTest(Sensor_ts *sensor) {
    TLx493D_A1B6_setBitfield(sensor, PT, TLx493D_A1B6_PARITY_TEST_DISABLE);
    TLx493D_A1B6_calculateParity(sensor);
    return TLx493D_A1B6_transferWriteRegisters(sensor);
}

void TLx493D_A1B6_calculateMagneticField(Sensor_ts *sensor, double *x, double *y, double *z) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    tlx493d_concatBytes(sensor, BX_MSB, BX_LSB, &valueX);
    tlx493d_concatBytes(sensor, BY_MSB, BY_LSB, &valueY);
    tlx493d_concatBytes(sensor, BZ_MSB, BZ_LSB, &valueZ);

    *x = ((double) valueX) * GEN_1_MAG_FIELD_MULT;
    *y = ((double) valueY) * GEN_1_MAG_FIELD_MULT;
    *z = ((double) valueZ) * GEN_1_MAG_FIELD_MULT;
}

bool TLx493D_A1B6_getMagneticField(Sensor_ts *sensor, double *x, double *y, double *z) {
    if( TLx493D_A1B6_readRegisters(sensor) ) {
        TLx493D_A1B6_calculateMagneticField(sensor, x, y, z);
        return true;
    }
    return false;
}

void TLE493D_A2B6_calculateMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp) {
    TLx493D_A1B6_calculateMagneticField(sensor, x, y, z);
    TLx493D_A1B6_calculateTemperature(sensor, temp);
}

bool TLE493D_A2B6_getMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp) {
    if( TLx493D_A1B6_readRegisters(sensor) ) {
        TLE493D_A2B6_calculateMagneticFieldAndTemperature(sensor, x, y, z, temp);
        return true;
    }
    return false;
}

bool TLx493D_A1B6_setIICAddress(Sensor_ts *sensor, TLx493D_StandardIICAddresses_te addr) {
    uint8_t bitfieldValue = 0;
    uint8_t deviceAddress = 0;

    switch (addr) {
        case GEN_1_STD_IIC_ADDR_00:
            bitfieldValue = 0b00;
            deviceAddress = 0xBC;
            break;

        case GEN_1_STD_IIC_ADDR_01:
            bitfieldValue = 0b01;
            deviceAddress = 0xB4;
            break;

        case GEN_1_STD_IIC_ADDR_10:
            bitfieldValue = 0b10;
            deviceAddress = 0x9C;
            break;

        case GEN_1_STD_IIC_ADDR_11:
            bitfieldValue = 0b11;
            deviceAddress = 0x94;
            break;
        
        default:
            return false;
    }

    TLx493D_A1B6_setBitfield(sensor, IICADR, bitfieldValue);
    TLx493D_A1B6_calculateParity(sensor);
    bool ret = TLx493D_A1B6_transferWriteRegisters(sensor);

    setI2CParameters(sensor, deviceAddress);

    return ret;
}

bool TLx493D_A1B6_enableInterrupt(Sensor_ts *sensor) {
    TLx493D_A1B6_setBitfield(sensor, INT, TLx493D_A1B6_INT_ENABLE_default);
    TLx493D_A1B6_calculateParity(sensor);

    return TLx493D_A1B6_transferWriteRegisters(sensor);
}


bool TLx493D_A1B6_disableInterrupt(Sensor_ts *sensor) {
    TLx493D_A1B6_setBitfield(sensor, INT, TLx493D_A1B6_INT_DISABLE);
    TLx493D_A1B6_calculateParity(sensor);

    return TLx493D_A1B6_transferWriteRegisters(sensor);
}

bool TLx493D_setLowPowerPeriod(Sensor_ts *sensor, TLx493D_A1B6_Reg_LOW_POWER_PERIOD_t lp_period){

    TLx493D_A1B6_setBitfield(sensor, LP, lp_period);
    TLx493D_A1B6_calculateParity(sensor);
    bool ret = TLx493D_A1B6_transferWriteRegisters(sensor);
    
    return ret;                                                 
}

bool TLx493D_A1B6_setPowerMode(Sensor_ts *sensor, TLx493D_A1B6_PowerMode_t mode){
    
    TLx493D_A1B6_setBitfield(sensor, FAST, TLx493D_A1B6_PowerModeCombinations[mode].FAST);
    TLx493D_A1B6_setBitfield(sensor, LOW_POWER, TLx493D_A1B6_PowerModeCombinations[mode].LOW_POWER);
    TLx493D_A1B6_setBitfield(sensor, LP, TLx493D_A1B6_PowerModeCombinations[mode].LP);
    TLx493D_A1B6_calculateParity(sensor);
    bool ret = TLx493D_A1B6_transferWriteRegisters(sensor);

    return ret;
}

// bool TLx493D_A1B6_transferRegisterMap(Sensor_ts *sensor, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len) {
    
//     return sensor->comLibIF->transfer.i2c_transfer(sensor, tx_buffer, tx_len, rx_buffer, rx_len);
// }

//todo: remove if not used. Note: So far not used 
void TLx493D_A1B6_getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue) {
    Register_ts *bf = &sensor->regDef[bitField];

    if(bf->accessMode == READ_MODE_e) {
        *bitFieldValue = (sensor->regMap[bf->address] & bf->mask) >> bf->offset;
    }

    assert(bf->accessMode == READ_MODE_e);
}

uint8_t TLx493D_A1B6_returnBitfield(Sensor_ts *sensor, uint8_t bitField) {
    Register_ts *bf = &sensor->regDef[bitField];
    uint8_t bitFieldValue;

    if(bf->accessMode == READ_MODE_e) {
        bitFieldValue = (sensor->regMap[bf->address] & bf->mask) >> bf->offset;
    }

    assert(bf->accessMode == READ_MODE_e);

    return bitFieldValue;
}

void TLx493D_A1B6_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue) {
    Register_ts *bf = &sensor->regDef[bitField];
    
    if(bf->accessMode == WRITE_MODE_e) {
        sensor->regMap[bf->address + GEN_1_WRITE_REGISTERS_OFFSET] = (sensor->regMap[bf->address + GEN_1_WRITE_REGISTERS_OFFSET] & ~bf->mask) | (newBitFieldValue << bf->offset);
    }

    assert(bf->accessMode == WRITE_MODE_e);
}

bool TLx493D_A1B6_writeRegister(Sensor_ts* sensor, uint8_t bitField) {
    bool ret = false;
    Register_ts *bf = &sensor->regDef[bitField];

    if(bf->accessMode == WRITE_MODE_e){
        uint8_t transBuffer[2] = { bf->address, sensor->regMap[bf->address] };

        ret = transfer(sensor, transBuffer, sizeof(transBuffer), NULL, 0);
        // ret = sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, sizeof(transBuffer), NULL, 0);
    }

    assert(bf->accessMode == WRITE_MODE_e);

    return ret;
}

//TODO: rename to transferreadregister to match common functions
bool TLx493D_A1B6_readRegisters(Sensor_ts *sensor) {
    return transfer(sensor, NULL, 0, sensor->regMap, TLx493D_A1B6_READ_REGISTERS_MAX_COUNT);
    // return sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, sensor->regMap, TLx493D_A1B6_READ_REGISTERS_MAX_COUNT);
}

bool TLx493D_A1B6_hasValidFuseParity(Sensor_ts *sensor){
    Register_ts *bf = &sensor->regDef[FF];
    return ((sensor->regMap[bf->address] & bf->mask) != 0);
}

bool TLx493D_A1B6_isFunctional(Sensor_ts *sensor){
    return TLx493D_A1B6_hasValidFuseParity(sensor);
}

bool TLx493D_A1B6_hasValidTBit(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[T];
    return ((sensor->regMap[bf->address] & bf->mask) == 0);
}

bool TLx493D_A1B6_hasValidPDBit(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[PD];
    return ((sensor->regMap[bf->address] & bf->mask) != 0);
}

bool TLx493D_A1B6_hasValidData(Sensor_ts *sensor){
    return TLx493D_A1B6_hasValidTBit(sensor) && TLx493D_A1B6_hasValidPDBit(sensor);
}
