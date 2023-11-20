
// std includes

// Arduino includes

// project cpp includes
#include "TLx493D_inc.hpp"


// SPI chip
#define POWER_PIN_LOW 3


TLx493D_P3I8 dut(SPI);


// TLx493D_P3I8 p3i8(SPI);


void setup() {
    pinMode(POWER_PIN_LOW, OUTPUT);
    digitalWrite(POWER_PIN_LOW, HIGH);

    delay(3000);
    Serial.begin(115200);
    delay(100);

    dut.begin();

    // p3i8.begin();

    delay(100);
    Serial.print("setup done.\n");
}


void loop() {
    double temp = 0.0;
    double valX = 0, valY = 0, valZ = 0;

    digitalWrite(POWER_PIN_LOW, LOW);
    Serial.print(true == dut.getTemperature(&temp) ? "getTemperature ok\n" : "getTemperature error\n");
    digitalWrite(POWER_PIN_LOW, HIGH);

    Serial.print("Temperature is: ");
    Serial.print(temp);
    Serial.println("°C");

    digitalWrite(POWER_PIN_LOW, LOW);
    Serial.print(true == dut.getMagneticField(&valX, &valY, &valZ) ? "getMagneticField ok\n" : "getMagneticField error\n");
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

    // Serial.print(true == dut.isWakeUpActive() ? "isWakeUpActive ok\n" : "isWakeUpActive error\n");

    printRegisters(dut.getSensor());
    Serial.print("\n");

    delay(1000);
}
