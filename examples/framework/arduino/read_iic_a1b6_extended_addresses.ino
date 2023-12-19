// std includes

// Arduino includes

// project cpp includes
#include "TLx493D_inc.hpp"

#define POWERPIN 8
#define SDA_ADDR_PIN 7 

// address 0x3E when SDA/ADDR held low at power up
TLx493D_A1B6 dut(Wire, TLx493D_IIC_ADDR_A4_e); //0x3E

void setup() {
    delay(3000);
    Serial.begin(115200);

    // explicit power pin needed to power up board after the addrPin is pulled down
    dut.setPowerPin(POWERPIN, OUTPUT, HIGH, LOW, 50, 50);
    // set pin used to drive SDA/ADDR pin before power up
    // This pin is then isolated from the I2C bus by switching it high-Z before I2C init
    dut.setAddrPin(SDA_ADDR_PIN, OUTPUT, LOW, HIGH, 1, 1);
    
    // Set the sensor constructor to activate extended address switching pin
    dut.begin(true, false, true);

    // Options to set the other remaining addresses when SDA low at power-up.
    // dut.setIICAddress(TLx493D_IIC_ADDR_A5_e); // 0x36
    // dut.setIICAddress(TLx493D_IIC_ADDR_A6_e); // 0x1E
    // dut.setIICAddress(TLx493D_IIC_ADDR_A7_e); // 0x16
  
    Serial.print("setup done.\n");
}


void loop() {
    double temp = 0.0;
    double valX = 0, valY = 0, valZ = 0;

    Serial.print(true == dut.getTemperature(&temp) ? "getTemperature ok\n" : "getTemperature error\n");

    Serial.print("Temperature is: ");
    Serial.print(temp);
    Serial.println("°C");

    Serial.print(true == dut.getMagneticField(&valX, &valY, &valZ) ? "getMagneticField ok\n" : "getMagneticField error\n");

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
