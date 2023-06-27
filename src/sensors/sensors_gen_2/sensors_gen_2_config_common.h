#ifndef SENSORS_GEN_2_CONFIG_COMMON_H
#define SENSORS_GEN_2_CONFIG_COMMON_H


// std includes
#include <stdint.h>


// General defines
#define GEN_2_MAG_FIELD_MULT        0.13
#define GEN_2_TEMP_MULT             0.24
#define GEN_2_TEMP_OFFSET           1180.0
#define GEN_2_TEMP_REF              25.0

#define GEN_2_STD_IIC_ADDR          0x35

// A0
#define GEN_2_STD_IIC_ADDR_WRITE_A0    0x6A
#define GEN_2_STD_IIC_ADDR_READ_A0     0x6B


#define GEN_2_REG_NUMBER            23

// #define GEN_2_REG_MASK_READ         0
// #define GEN_2_REG_MASK_WRITE        1
// #define GEN_2_REG_MASK_READ_WRITE   2

// typedef enum {
//     CP_F,
//     FP_F,
//     P_F
// } parityFlags_t;

// typedef enum {
//     BXMSB = 0,
//     BYMSB,
//     BZMSB,
//     TEMPMSB,
//     BXLSB, BYLSB,
//     TEMPLSB, ID, BZLSB,
//     P, FF, CF, T, PD3, PD0, FRM,
//     XLMSB,
//     XHMSB,
//     YLMSB,
//     YHMSB,
//     ZLMSB,
//     ZHMSB,
//     WA, WU, XHLSB, XLLSB,
//     TST, YHLSB, YLLSB,
//     PH, ZHLSB, ZLLSB,
//     DT, AM, TRIG, X2, TLMAG, CP,
//     FP, IICADR, PR, CA, INT, MODE,
//     PRD,
//     X4,
//     TYPE, HWV
// } registers_t;

// typedef struct {
//     uint8_t rw;
//     uint8_t reg_address;
//     uint8_t bit_mask;
//     uint8_t shift;
// } reg_mask_t;

// extern const reg_mask_t register_mask[];


#endif // SENSORS_GEN_2_CONFIG_COMMON_H
