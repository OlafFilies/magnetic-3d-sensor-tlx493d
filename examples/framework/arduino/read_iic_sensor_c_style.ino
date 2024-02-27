/** Project CPP includes */
#include "TLx493D_inc.hpp"


const uint8_t POWER_PIN = LED2;


TLx493D_t dut;


void setup() {
    Serial.begin(115200);
    delay(3000);

    /** Setting the defined power up pin as input and enabling it */
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, HIGH);

    /** Initialization of the sensor and the communication interface.
     *  After that the default configuration of the sensor is set. This means
     *  1-Byte read mode and interrupt disabled.
     */
    tlx493d_init(&dut, TLx493D_P2B6_e);
    ifx::tlx493d::initCommunication(&dut, Wire, TLx493D_IIC_ADDR_A0_e);
    tlx493d_setDefaultConfig(&dut);

    Serial.println("setup done.");
}

/** In the loop we continuously reading the temperature value as well as the
 *  magnetic values in X, Y, Z-direction of the sensor and printing them to
 *  the serial monitor
 */
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

    tlx493d_printRegisters(dut.getSensor());
    Serial.print("\n");

    delay(1000);
}