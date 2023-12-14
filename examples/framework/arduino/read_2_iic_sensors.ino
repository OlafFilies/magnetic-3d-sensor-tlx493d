// std includes

// Arduino includes

// project cpp includes
#include "TLx493D_inc.hpp"


// TLx493D_A1B6 dut(Wire);

// TLx493D_A2B6 dut(Wire);
// TLx493D_P2B6 dut(Wire);
TLx493D_W2B6 dut(Wire, TLx493D_IIC_ADDR_A0_e);
// TLx493D_W2BW dut(Wire);

// TLx493D_P3B6 dut(Wire);


// TODO: if A1B6 instantiated program not working
// TLx493D_A1B6 a1b6(Wire);

// TLx493D_A2B6 a2b6(Wire);
// TLx493D_P2B6 p2b6(Wire);
// TLx493D_W2B6 w2b6(Wire);
// TLx493D_W2BW w2bw(Wire);

// TLx493D_P3B6 p3b6(Wire);


void setup() {
    delay(3000);
    Serial.begin(115200);
    delay(100);

    dut.setPowerPin(LED2, OUTPUT, HIGH, LOW, 50, 50);
    dut.begin();
    // dut.enablePower();


    // a1b6.begin();

    // a2b6.begin();
    // p2b6.begin();
    // w2b6.begin();
    // w2bw.begin();

    // p3b6.begin();


    delay(100);
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
