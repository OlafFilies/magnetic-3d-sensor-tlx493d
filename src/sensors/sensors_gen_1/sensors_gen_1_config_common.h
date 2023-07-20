#ifndef SENSORS_GEN_1_CONFIG_COMMON_H
#define SENSORS_GEN_1_CONFIG_COMMON_H

// General defines
#define GEN_1_STD_IIC_ADDR          0xBC //remove: this is then >>1 while passing to seti2cparams, 
                                         // there it becomes 0x1F, which then is <<1 and concat'd with R/W bit     

#endif // SENSORS_GEN_1_CONFIG_COMMON_H
