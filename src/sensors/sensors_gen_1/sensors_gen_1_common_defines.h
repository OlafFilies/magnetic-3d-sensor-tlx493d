#ifndef SENSORS_GEN_1_CONFIG_COMMON_H
#define SENSORS_GEN_1_CONFIG_COMMON_H

// General defines
#define GEN_1_TEMP_MULT                     1.10
#define GEN_1_TEMP_OFFSET                   315.0

#define GEN_1_MAG_FIELD_MULT                0.098

#define GEN_1_STD_IIC_ADDR                  0xBC //remove: this is then >>1 while passing to seti2cparams, 
                                            // there it becomes 0x1F, which then is <<1 and concat'd with R/W bit

#define GEN_1_WRITE_REGISTERS_OFFSET        10

#endif // SENSORS_GEN_1_CONFIG_COMMON_H
