#ifndef SENSORS_COMMON_H
#define SENSORS_COMMON_H


#include <stdbool.h>
#include <stdint.h>


bool      init(void *sensor);
bool      deinit(void *sensor);
uint32_t  getTemperature(void *sensor);
bool      getFieldValues(void *sensor, uint32_t *x, uint32_t *y, uint32_t *z);


#endif // SENSORS_COMMON_H
