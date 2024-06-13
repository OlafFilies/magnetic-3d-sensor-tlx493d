/** Project CPP includes. */
#include "TLx493D_inc.hpp"


/** Definition of the power pin. */
const uint8_t POWER_PIN = LED2;


/** Declaration of the sensor. */
TLx493D_t dut;

uint8_t count = 0;
TLx493D_SupportedSensorType_t sensorType = TLx493D_A2B6_e;


void setup() {
    Serial.begin(115200);
    delay(3000);

    tlx493d_init(&dut, sensorType);

    /** Setting the defined power up pin as output and set to VDD / high value. */
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, HIGH);

    /** Initialization of the sensor and the communication interface.
     *  After that the default configuration of the sensor is set. This means
     *  1-Byte read mode and interrupt disabled.
     */
    ifx::tlx493d::initCommunication(&dut, Wire, TLx493D_IIC_ADDR_A0_e, true);
    tlx493d_setDefaultConfig(&dut);

    Serial.println("setup done.");
}

/** In the loop we continuously reading the temperature value as well as the
 *  magnetic values in X, Y, Z-direction of the sensor and printing them to
 *  the serial monitor.
 */
void loop() {
    Serial.println("loop ...");

    double temp = 0.0;
    double valX = 0;
    double valY = 0;
    double valZ = 0;

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

    tlx493d_printRegisters(&dut);
    Serial.print("\n");

    delay(1000);

    Serial.print("count : ");
    Serial.println(count);

    if( ++count == 5 ) {
        Serial.println("\nBefore reset -------------------------------------------------------------------------------------------------------");

        // If reset fails for A1B6 and W2BW : check drive strength of power pin !
        ifx::tlx493d::deinitCommunication(&dut, sensorType != TLx493D_A1B6_e);
        dut.functions->deinit(&dut);

        Serial.println("\npower low --------");
        digitalWrite(POWER_PIN, LOW);
        delayMicroseconds(250000);

        tlx493d_init(&dut, sensorType);

        Serial.println("\npower high --------");
        digitalWrite(POWER_PIN, HIGH);
        delayMicroseconds(0);

        ifx::tlx493d::initCommunication(&dut, Wire, TLx493D_IIC_ADDR_A0_e, true);
        tlx493d_setDefaultConfig(&dut);

        Serial.println("\nAfter reset -------------------------------------------------------------------------------------------------------");
        count = 0;
    }
}
