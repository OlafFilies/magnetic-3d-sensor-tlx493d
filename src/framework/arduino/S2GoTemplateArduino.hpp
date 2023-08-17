
#ifndef S2GO_TEMPLATE_ARDUINO_HPP
#define S2GO_TEMPLATE_ARDUINO_HPP


// std include

// Arduino includes
#include <Arduino.h>

// project cpp includes
#include "arduino_defines.h"
#include "S2GoTemplate.hpp"


// #include "gpio.h"
// #include "xmc_gpio.h"
// #include "xmc1_gpio_map.h"
// #include "xmc1_gpio.h"
// #include "src/xmc1100/i2c/conf_i2c.h"
// #include "src/xmc1100/i2c/i2c.h"
// #include "src/xmc1100/time/time.h"




typedef struct pinCtrl {
    uint8_t pinNumber;
    uint8_t direction;
    uint8_t enableValue;
    uint8_t disableValue;
} pinCtrl;


template<> class S2GoTemplate<pinCtrl> {
   public:

        S2GoTemplate() : constantPins{LED2, OUTPUT, HIGH, LOW}, switchedPins{} {
        }

        void init() {
            // using global function to demonstrate usage
            preTransferHook  = myPreTransferHook;
            postTransferHook = myPostTransferHook;


	    // NVIC_DisableIRQ(ERU0_0_IRQn);

        // XMC_GPIO_CONFIG_t cfg;
   		// cfg.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,

		// cfg.output_level = XMC_GPIO_OUTPUT_LEVEL_LOW;
		// XMC_GPIO_Init(P1_0, &cfg);

		// cfg.output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH;
		// XMC_GPIO_Init(P0_15, &cfg);

		// // delay for 200us
		// delayMicroseconds(5000 - 2);

		// cfg.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
		// cfg.output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH;
		// XMC_GPIO_Init(P1_0, &cfg);

		// cfg.output_level = XMC_GPIO_OUTPUT_LEVEL_LOW;
		// XMC_GPIO_Init(P0_15, &cfg);

		// delayMicroseconds(2);


            for(auto &p : constantPins) {
                pinMode(p.pinNumber, p.direction);
                digitalWrite(p.pinNumber, p.enableValue);
            }

            for(auto &p : switchedPins) {
                pinMode(p.pinNumber, p.direction);
                digitalWrite(p.pinNumber, p.disableValue);
            }
        }


        void deinit() {
            for(auto &p : constantPins) {
                digitalWrite(p.pinNumber, p.disableValue);
            }

            disable();
        }


        void enable() {
            preTransferHook();

            for(auto &p : switchedPins) {
                digitalWrite(p.pinNumber, p.enableValue);
            }
        }


        void disable() {
            postTransferHook();

            for(auto &p : switchedPins) {
                digitalWrite(p.pinNumber, p.disableValue);
            }
        }


    private:

        hookFunctionType preTransferHook;
        hookFunctionType postTransferHook;

        pinCtrl constantPins[1];
        pinCtrl switchedPins[0];
};


typedef S2GoTemplate<pinCtrl> S2GoTemplateArduino;


#endif // S2GO_TEMPLATE_ARDUINO_HPP
