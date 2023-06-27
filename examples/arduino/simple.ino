
// std includes

// Arduino includes
#include <SPI.h>
#include <Arduino.h>

// project cpp includes
#include "S2GoTemplateArduino.hpp"
#include "TLx493D.hpp"
#include "TwoWire_Lib.hpp"
#include "Magnetic_3d.hpp"


extern "C" {
  #include "sensor_types.h"
}


void myPreTransferHook() {
}


void myPostTransferHook() {
}


Sensor3D<S2GoTemplateArduino, TwoWire_Lib, TwoWire, TLx493D> sensor3d(Wire, TLE493D_A2B6_e, I2C_e);


void setup() {
    Serial.begin(115200);
    delay(200);

    sensor3d.begin();

    delay(1000);
    Serial.print("setup done.\n");
}

void loop() {
    float temp = 0.0;
    float valX = 0, valY = 0, valZ = 0;

    // sensor3d.getTemperature(&temp);
    Serial.print(true == sensor3d.getTemperature(&temp) ? "getTemperature ok\n" : "getTemperature error\n");
    Serial.print("Temperature is: ");
    Serial.print(temp);
    Serial.println("°C");

    // sensor3d.updateGetFieldValues(&valX, &valY, &valZ);
    // sensor3d.getFieldValues(&valX, &valY, &valZ);
    Serial.print(true == sensor3d.getFieldValues(&valX, &valY, &valZ) ? "getFieldValues ok\n" : "getFieldValues error\n");
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
