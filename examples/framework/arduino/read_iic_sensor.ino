/** Project CPP includes */
#include "TLx493D_inc.hpp"


using namespace ifx::tlx493d;


/* For XMC 1100 boards. */
const uint8_t POWER_PIN = LED2;

TLx493D_A1B6 dut(Wire, TLx493D_IIC_ADDR_A0_e);

// TLx493D_A2B6 dut(Wire, TLx493D_IIC_ADDR_A0_e);
// TLx493D_P2B6 dut(Wire, TLx493D_IIC_ADDR_A0_e);
// TLx493D_W2B6 dut(Wire, TLx493D_IIC_ADDR_A0_e);


/* for XMC 4700 boards */
// const uint8_t POWER_PIN = 8; // P1.10

// TLx493D_W2BW dut(Wire, TLx493D_IIC_ADDR_A0_e);

// // TLx493D_P3B6 dut(Wire, TLx493D_IIC_ADDR_A0_e);
// TLx493D_P3B6 dut(Wire, TLx493D_IIC_ADDR_A1_e);


uint8_t count = 0;


void setup() {
    Serial.begin(115200);
    delay(3000);

    /** Definition of the power pin to power up the sensor. */
    dut.setPowerPin(POWER_PIN, OUTPUT, INPUT, HIGH, LOW, 0, 250000);
    // dut.setPowerPin(POWER_PIN, OUTPUT, HIGH, LOW, 250000, 250000);
    dut.begin();

    Serial.print("setup done.\n");
}

/** In the loop we continuously reading the temperature value as well as the
 *  magnetic values in X, Y, Z-direction of the sensor and printing them to
 *  the serial monitor
 */
void loop() {
    double t, x, y, z;

    dut.setSensitivity(TLx493D_FULL_RANGE_e);
    dut.getMagneticFieldAndTemperature(&x, &y, &z, &t);
    dut.printRegisters();

    Serial.print("Temperature is: ");
    Serial.print(t);
    Serial.println("°C");

    Serial.print("Value X is: ");
    Serial.print(x);
    Serial.println(" mT");
    Serial.print("Value Y is: ");
    Serial.print(y);
    Serial.println(" mT");
    Serial.print("Value Z is: ");
    Serial.print(z);
    Serial.println(" mT");

    dut.setSensitivity(TLx493D_SHORT_RANGE_e);
    dut.getMagneticFieldAndTemperature(&x, &y, &z, &t);
    dut.printRegisters();

    Serial.print("Temperature is: ");
    Serial.print(t);
    Serial.println("°C");

    Serial.print("Value X is: ");
    Serial.print(x);
    Serial.println(" mT");
    Serial.print("Value Y is: ");
    Serial.print(y);
    Serial.println(" mT");
    Serial.print("Value Z is: ");
    Serial.print(z);
    Serial.println(" mT");
    Serial.print("\n\n\n\n");

    delay(1000);

    Serial.print("count : ");
    Serial.println(count);

    if( ++count == 2 ) {
        Serial.println("\nBefore reset -------------------------------------------------------------------------------------------------------");
        // reset does not work for W2BW : either drive strength too low or delay to stabilize critical.
        // reset also causes issues with A1B6.
        dut.reset();
        Serial.println("\nAfter reset -------------------------------------------------------------------------------------------------------");
        count = 0;
    }
}
