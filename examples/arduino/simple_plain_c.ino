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


void setup() {
    Serial.begin(115200);
    delay(100);

    // required for S2Go
    // pinMode(LED2, OUTPUT);
    // digitalWrite(LED2, HIGH);

    init(&a2b6, TLE493D_A2B6_e, I2C_e);
    initI2CComLibIF(&a2b6, Wire);
    
    // reset(&a2b6);
    setDefaultConfig(&a2b6);

    delay(100);
    Serial.print("setup done.\n");
}


void loop() {
    float temp = 0.0;
    float valX = 0, valY = 0, valZ = 0;

    Serial.print(true == updateGetTemperature(&a2b6, &temp) ? "getTemperature ok\n" : "getTemperature error\n");
    Serial.print("Temperature is: ");
    Serial.print(temp);
    Serial.println("°C");

    Serial.print(true == updateGetFieldValues(&a2b6, &valX, &valY, &valZ) ? "updateGetFieldValues ok\n" : "updateGetFieldValues error\n");
    Serial.print("Value X is: ");
    Serial.print(valX);
    Serial.println(" mT");
    Serial.print("Value Y is: ");
    Serial.print(valY);
    Serial.println(" mT");
    Serial.print("Value Z is: ");
    Serial.print(valZ);
    Serial.println(" mT");

    delay(1000);
}