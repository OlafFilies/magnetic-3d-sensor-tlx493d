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


// #define REGISTER_SIZE_IN_BITS       (8)


#endif // SENSORS_GEN_2_CONFIG_COMMON_H
