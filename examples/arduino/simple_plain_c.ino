// std includes

// Arduino includes
#include <SPI.h>
#include <Arduino.h>

// project cpp includes
#include "arduino_defines.h"


extern "C" {
    #include "sensor_types.h"
    #include "sensors_common.h"
}


Sensor_ts a2b6;
struct I2C_t iic = { .wire = &Wire };


void setup() {
    Serial.begin(115200);
    delay(1000);

    init(&a2b6, TLE493D_A2B6_e, I2C_e);
    
//    reset(&a2b6);
    
    initComLibIF(&a2b6, iic);
    
    setDefaultConfig(&a2b6);

    delay(1000);
    Serial.print("setup done.\n");
}


void loop() {
    float temp = 0.0;
    float valX = 0, valY = 0, valZ = 0;

    updateGetTemperature(&a2b6, &temp);
    Serial.print("Temperature is: ");
    Serial.print(temp);
    Serial.println("°C");

    updateGetFieldValues(&a2b6, &valX, &valY, &valZ);
    Serial.print("Value X is: ");
    Serial.print(valX);
    Serial.println(" mT");
    Serial.print("Value Y is: ");
    Serial.print(valY);
    Serial.println(" mT");
    Serial.print("Value Z is: ");
    Serial.print(valZ);
    Serial.println(" mT");
    delay(2000);
}