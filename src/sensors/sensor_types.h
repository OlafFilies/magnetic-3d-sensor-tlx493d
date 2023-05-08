#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H


typedef enum  { TLE493D_A1B6 = 0, 
                TLV493D_A1B6 = 1,
                TLE493D_P2B6 = 2,
                TLE493D_W2B6 = 3 } sensorTypes_t;


typedef struct I2C_t I2C_t;


void printType(sensorTypes_t sensorType);


#endif // SENSOR_TYPES_H