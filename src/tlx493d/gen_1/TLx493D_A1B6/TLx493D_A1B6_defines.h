#ifndef TLx493D_A1B6_DEFINES_H
#define TLx493D_A1B6_DEFINES_H

// TODO: change prefix from  GEN_1 to TLx493D_A1B6
// TODO: remove unused defines
// #define TLx493D_A1B6_WRITE_REGISTER_MAP_SIZE        13          
// #define TLx493D_A1B6_READ_REGISTER_MAP_SIZE         8         
// #define TLx493D_A1B6_REGISTER_MAP_SIZE              (TLx493D_A1B6_WRITE_REGISTER_MAP_SIZE + TLx493D_A1B6_READ_REGISTER_MAP_SIZE)
#define TLx493D_A1B6_WRITE_REGISTERS_MAX_COUNT      4
#define TLx493D_A1B6_WRITE_REGISTERS_OFFSET         10

#define GEN_1_REG_MAP_SIZE                          14

#define TLx493D_A1B6_READ_REGISTERS_MAX_COUNT       10 //TODO: rename with consistent naming once we decide which parts stay in common and which go into sensor src file

#define GEN_1_WRITE_REGISTERS_OFFSET                10

// General defines
#define GEN_1_TEMP_MULT                             1.10
#define GEN_1_TEMP_OFFSET                           315.0  // TODO: should be 340 ! And offset is 25 !

#define GEN_1_MAG_FIELD_MULT                        0.098

#define GEN_1_STD_IIC_ADDR                          0xBC 

// A0
#define GEN_1_STD_IIC_ADDR_WRITE_A0       0xBC
// #define GEN_2_STD_IIC_ADDR_READ_A0        0xBD

// TODO: verify addresses !
// A1
#define GEN_1_STD_IIC_ADDR_WRITE_A1       0xB4
// #define GEN_2_STD_IIC_ADDR_READ_A1        0x

// A2
#define GEN_1_STD_IIC_ADDR_WRITE_A2       0x9C
// #define GEN_2_STD_IIC_ADDR_READ_A2        0x

// A3
#define GEN_1_STD_IIC_ADDR_WRITE_A3       0x94
// #define GEN_2_STD_IIC_ADDR_READ_A3        0x


#define GEN_1_BITFIELDS_COUNT                       28


#endif // TLx493D_A1B6_DEFINES_H
