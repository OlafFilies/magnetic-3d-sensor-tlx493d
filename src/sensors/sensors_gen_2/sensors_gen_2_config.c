// project c includes
// common to same generation of sensors
#include "sensors_gen_2_config_common.h"


// const reg_mask_t register_mask[] = {
//     // BX
//     {GEN_2_REG_MASK_READ,       0x00, 0xFF, 0},   // BXMSB
//     // BY
//     {GEN_2_REG_MASK_READ,       0x01, 0xFF, 0},   // BYMSB
//     // BZ
//     {GEN_2_REG_MASK_READ,       0x02, 0xFF, 0},   // BZMSB
//     // TEMP
//     {GEN_2_REG_MASK_READ,       0x03, 0xFF, 0},   // TEMPMSB
//     // BX2
//     {GEN_2_REG_MASK_READ,       0x04, 0xF0, 4},   // BXLSB
//     {GEN_2_REG_MASK_READ,       0x04, 0x0F, 0},   // BYLSB
//     // TEMP2
//     {GEN_2_REG_MASK_READ,       0x05, 0xC0, 6},   // TEMPLSB
//     {GEN_2_REG_MASK_READ,       0x05, 0x30, 4},   // ID
//     {GEN_2_REG_MASK_READ,       0x05, 0x0F, 0},   // BZLSB
//     // DIAG
//     {GEN_2_REG_MASK_READ,       0x06, 0x80, 7},   // P
//     {GEN_2_REG_MASK_READ,       0x06, 0x40, 6},   // FF
//     {GEN_2_REG_MASK_READ,       0x06, 0x20, 5},   // CF
//     {GEN_2_REG_MASK_READ,       0x06, 0x10, 4},   // T
//     {GEN_2_REG_MASK_READ,       0x06, 0x08, 3},   // PD3
//     {GEN_2_REG_MASK_READ,       0x06, 0x04, 2},   // PD0
//     {GEN_2_REG_MASK_READ,       0x06, 0x03, 0},   // FRM
//     // XL
//     {GEN_2_REG_MASK_READ_WRITE, 0x07, 0xFF, 0},   // XLMSB
//     // XH
//     {GEN_2_REG_MASK_READ_WRITE, 0x08, 0xFF, 0},   // XHMSB
//     // YL
//     {GEN_2_REG_MASK_READ_WRITE, 0x09, 0xFF, 0},   // YLMSB
//     // YH
//     {GEN_2_REG_MASK_READ_WRITE, 0x0A, 0xFF, 0},   // YHMSB
//     // ZL
//     {GEN_2_REG_MASK_READ_WRITE, 0x0B, 0xFF, 0},   // ZLMSB
//     // ZH
//     {GEN_2_REG_MASK_READ_WRITE, 0x0C, 0xFF, 0},   // ZHMSB
//     // WU
//     {GEN_2_REG_MASK_READ,       0x0D, 0x80, 7},   // WA
//     {GEN_2_REG_MASK_READ_WRITE, 0x0D, 0x40, 6},   // WU
//     {GEN_2_REG_MASK_READ_WRITE, 0x0D, 0x38, 3},   // XHLSB
//     {GEN_2_REG_MASK_READ_WRITE, 0x0D, 0x07, 0},   // XLLSB
//     // YHL2
//     {GEN_2_REG_MASK_READ_WRITE, 0x0E, 0xC0, 6},   // TST
//     {GEN_2_REG_MASK_READ_WRITE, 0x0E, 0x38, 3},   // YHLSB
//     {GEN_2_REG_MASK_READ_WRITE, 0x0E, 0x07, 0},   // YLLSB
//     // ZHL2
//     {GEN_2_REG_MASK_READ_WRITE, 0x0F, 0xC0, 6},   // PH
//     {GEN_2_REG_MASK_READ_WRITE, 0x0F, 0x38, 3},   // ZHLSB
//     {GEN_2_REG_MASK_READ_WRITE, 0x0F, 0x07, 0},   // ZLLSB
//     // CONFIG
//     {GEN_2_REG_MASK_READ_WRITE, 0x10, 0x80, 7},   // DT
//     {GEN_2_REG_MASK_READ_WRITE, 0x10, 0x40, 6},   // AM
//     {GEN_2_REG_MASK_READ_WRITE, 0x10, 0x30, 4},   // TRIG
//     {GEN_2_REG_MASK_READ_WRITE, 0x10, 0x08, 3},   // X2
//     {GEN_2_REG_MASK_READ_WRITE, 0x10, 0x06, 1},   // TLMAG
//     {GEN_2_REG_MASK_READ_WRITE, 0x10, 0x01, 0},   // CP
//     // MOD1
//     {GEN_2_REG_MASK_READ_WRITE, 0x11, 0x80, 7},   // FP
//     {GEN_2_REG_MASK_READ_WRITE, 0x11, 0x60, 5},   // IICADR
//     {GEN_2_REG_MASK_READ_WRITE, 0x11, 0x10, 4},   // PR
//     {GEN_2_REG_MASK_READ_WRITE, 0x11, 0x08, 3},   // CA
//     {GEN_2_REG_MASK_READ_WRITE, 0x11, 0x06, 2},   // INT
//     {GEN_2_REG_MASK_READ_WRITE, 0x11, 0x03, 0},   // MODE
//     // MOD2
//     {GEN_2_REG_MASK_READ_WRITE, 0x13, 0xE0, 5},   // PRD
//     // CONFIG2
//     {GEN_2_REG_MASK_WRITE,      0x14, 0x01, 0},   // X4
//     // VER
//     {GEN_2_REG_MASK_READ,       0x16, 0x30, 4},   // TYPE
//     {GEN_2_REG_MASK_READ,       0x16, 0x0F, 0},   // HWV
// };