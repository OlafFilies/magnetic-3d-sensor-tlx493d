// std includes
#include <stdbool.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"

// common to same generation of sensors

// sensor specific includes

// project cpp includes


extern "C" bool transferSPI(Sensor_ts *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen);
extern "C" bool transferIIC(Sensor_ts *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen);


extern "C" bool transfer(Sensor_ts *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen) {
    return sensor->comIFType == I2C_e ? transferIIC(sensor, txBuffer, txLen, rxBuffer, rxLen)
                                      : (sensor->comIFType == SPI_e ? transferSPI(sensor, txBuffer, txLen, rxBuffer, rxLen)
                                                                    : false);
}