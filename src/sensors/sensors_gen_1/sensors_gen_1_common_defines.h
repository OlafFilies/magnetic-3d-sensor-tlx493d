#ifndef SENSORS_GEN_1_CONFIG_COMMON_H
#define SENSORS_GEN_1_CONFIG_COMMON_H

// General defines
#define GEN_1_TEMP_MULT                             1.10
#define GEN_1_TEMP_OFFSET                           315.0

#define GEN_1_MAG_FIELD_MULT                        0.098

#define GEN_1_STD_IIC_ADDR                          0xBC 

#define GEN_1_WRITE_REGISTERS_OFFSET                10

#define GEN_1_REG_MAP_SIZE                          14

#define TLx493D_A1B6_READ_REGISTERS_MAX_COUNT       10 //TODO: rename with consistent naming once we decide which parts stay in common and which go into sensor src file

#endif // SENSORS_GEN_1_CONFIG_COMMON_H
