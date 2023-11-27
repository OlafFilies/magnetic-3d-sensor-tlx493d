// std includes

// // Arduino includes

// project cpp includes
#include "TLx493D_inc.hpp"


// S2Go boards
#define POWER_PIN_HIGH 15


TLx493D_t dut;


void setup() {
    Serial.begin(115200);
    delay(3000);

    // required for S2Go
    pinMode(POWER_PIN_HIGH, OUTPUT);
    digitalWrite(POWER_PIN_HIGH, HIGH);


    // tlx493d_init(&dut, TLx493D_A1B6_e);
    // tlx493d_init(&dut, TLx493D_A2B6_e);
    tlx493d_init(&dut, TLx493D_P2B6_e);
    // tlx493d_init(&dut, TLx493D_W2B6_e);
    // tlx493d_init(&dut, TLx493D_W2BW_e);
    // tlx493d_init(&dut, TLx493D_P3B6_e);

    tlx493d_initCommunication(&dut, Wire, TLx493D_IIC_ADDR_A0_e);
    tlx493d_setDefaultConfig(&dut);

    delay(100);
    Serial.println("setup done.");
}


void loop() {
    double temp = 0.0;
    double valX = 0, valY = 0, valZ = 0;
    Serial.println("loop ...");

    Serial.print(true == tlx493d_getTemperature(&dut, &temp) ? "getTemperature ok\n" : "getTemperature error\n");

    Serial.print("Temperature is: ");
    Serial.print(temp);
    Serial.println("°C");

    Serial.print(true == tlx493d_getMagneticField(&dut, &valX, &valY, &valZ) ? "getMagneticField ok\n" : "getMagneticField error\n");

    Serial.print("Value X is: ");
    Serial.print(valX);
    Serial.println(" mT");
    Serial.print("Value Y is: ");
    Serial.print(valY);
    Serial.println(" mT");
    Serial.print("Value Z is: ");
    Serial.print(valZ);
    Serial.println(" mT");

    // Serial.print(true == tlx493d_isFunctional(&dut) ? "isFunctional\n" : "NOT isFunctional\n");
    // Serial.print(true == tlx493d_hasValidData(&dut) ? "hasValidData\n" : "NOT hasValidData\n");
    // Serial.print(true == tlx493d_hasValidTemperatureData(&dut) ? "hasValidTemperatureData\n" : "NOT hasValidTemperatureData\n");
    // Serial.print(true == tlx493d_hasValidMagneticFieldData(&dut) ? "hasValidMagneticFieldData\n" : "NOT hasValidFieldData\n");


    // Serial.print(true == TLx493D_A2B6_hasValidIICadr(&dut) ? "TLx493D_A2B6_hasValidIICadr\n" : "NOT TLx493D_A2B6_hasValidIICadr\n");
    // Serial.print(true == tlx493d_hasWakeUp(&dut) ? "hasWakeup\n" : "NOT hasWakeup\n");


    // tlx493d_setIICAddress(&dut, GEN_2_STD_IIC_ADDR_A1);
    // Serial.print(dut.comLibIFParams.i2c_params.address << 1);
    // Serial.print(true == TLx493D_A2B6_hasValidIICadr(&dut) ? "TLx493D_A2B6_hasValidIICadr\n" : "NOT TLx493D_A2B6_hasValidIICadr\n");

    // tlx493d_setPowerMode(&dut, 0b00);
    // delay(1000);
    // tlx493d_setPowerMode(&dut, 0b01);
    // delay(1000);

    // TLx493D_A2B6_disableCollisionAvoidance(&dut);
    // tlx493d_setPowerMode(&dut, 0b11);
    // delay(1000);
    // TLx493D_A2B6_enableCollisionAvoidance(&dut);

    printRegisters(&dut);
    Serial.print("\n");

    delay(1000);
}