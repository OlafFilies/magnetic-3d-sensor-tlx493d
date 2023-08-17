
// std includes

// project cpp includes
#include "arduino_defines.h"
#include <xmc_i2c.h>

// project c includes



// void myPreTransferHook() {
// }


// void myPostTransferHook() {
// }


extern "C" void myDebug(char *msg) {
	__enable_irq();
    Serial.print("\nmsg : ");
    Serial.println(msg);
    Serial.flush();
	__disable_irq();
}
