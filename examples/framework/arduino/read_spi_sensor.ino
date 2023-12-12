
// std includes

// Arduino includes

// project cpp includes
#include "TLx493D_inc.hpp"


// SPI chip
#define POWER_PIN_LOW 3


TLx493D_P3I8 dut(SPI);


void setup() {
    delay(3000);
    Serial.begin(115200);

    dut.setPowerPin(LED2, OUTPUT, HIGH, LOW, 50, 50);
    dut.setSelectPin(POWER_PIN_LOW, OUTPUT, LOW, HIGH, 50, 50);
    dut.begin();

    Serial.print("setup done.\n");
}


void loop() {
    double temp = 0.0;
    double valX = 0, valY = 0, valZ = 0;
    int16_t tempRaw = 0;

    // dut.enableSelect();
    Serial.print(true == dut.getTemperature(&temp) ? "getTemperature ok\n" : "getTemperature error\n");
    // dut.disableSelect();

    Serial.print("Temperature is: ");
    Serial.print(temp);
    Serial.println(" °C");


    // dut.enableSelect();
    dut.getRawTemperature(&tempRaw);
    // dut.disableSelect();

    Serial.print("Raw temperature is: ");
    Serial.print(tempRaw);
    Serial.println(" LSB");

    // dut.enableSelect();
    Serial.print(true == dut.getMagneticField(&valX, &valY, &valZ) ? "getMagneticField ok\n" : "getMagneticField error\n");
    // dut.disableSelect();

    Serial.print("Value X is: ");
    Serial.print(valX);
    Serial.println(" mT");
    Serial.print("Value Y is: ");
    Serial.print(valY);
    Serial.println(" mT");
    Serial.print("Value Z is: ");
    Serial.print(valZ);
    Serial.println(" mT");

    printRegisters(dut.getSensor());
    Serial.print("\n");

    delay(1000);
}
