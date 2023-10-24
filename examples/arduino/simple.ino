
// std includes

// Arduino includes
#include <Arduino.h>

// project cpp includes
#include "arduino_defines.h"
#include "Magnetic_3d.hpp"
#include "S2GoTemplateArduino.hpp"
#include "SPI_Lib.hpp"
#include "TLx493D.hpp"
#include "TwoWire_Lib.hpp"


// SPI chip
#define POWER_PIN_LOW 3


extern "C" {
    #include "sensor_types.h"
    #include "sensors_gen_2_common_defines.h"
    #include "sensors_gen_2_common.h"
    #include "sensors_gen_3_common_defines.h"
    #include "sensors_gen_3_common.h"
    #include "sensors_common.h"

    #include "TLE493D_A2B6.h"
    #include "TLE493D_P2B6.h"
    #include "TLE493D_W2B6.h"

    #include "TLE493D_P3B6.h"
    #include "TLE493D_P3I8.h"
}


// Sensor3D<S2GoTemplateArduino, TwoWire_Lib, TwoWire, TLx493D> dut(Wire, TLE493D_A2B6_e, I2C_e);
Sensor3D<S2GoTemplateArduino, SPI_Lib, SPIClass, TLx493D> dut(SPI, TLE493D_P3I8_e, SPI_e);


void setup() {
    pinMode(POWER_PIN_LOW, OUTPUT);
    digitalWrite(POWER_PIN_LOW, HIGH);

    delay(3000);
    Serial.begin(115200);
    delay(100);

    dut.begin();

    delay(100);
    Serial.print("setup done.\n");
}

void loop() {
    float temp = 0.0;
    float valX = 0, valY = 0, valZ = 0;

    digitalWrite(POWER_PIN_LOW, LOW);
    Serial.print(true == dut.getTemperature(&temp) ? "getTemperature ok\n" : "getTemperature error\n");
    digitalWrite(POWER_PIN_LOW, HIGH);

    Serial.print("Temperature is: ");
    Serial.print(temp);
    Serial.println("°C");

    digitalWrite(POWER_PIN_LOW, LOW);
    Serial.print(true == dut.getFieldValues(&valX, &valY, &valZ) ? "getFieldValues ok\n" : "getFieldValues error\n");
    digitalWrite(POWER_PIN_LOW, HIGH);

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
