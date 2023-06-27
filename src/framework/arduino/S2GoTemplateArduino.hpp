
#ifndef S2GO_TEMPLATE_ARDUINO_HPP
#define S2GO_TEMPLATE_ARDUINO_HPP


// std include

// Arduino includes
#include <Arduino.h>

// project cpp includes
#include "arduino_defines.h"
#include "S2GoTemplate.hpp"


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

        // vector, list, ... do not work with current XMC compiler version and settings !!!
        pinCtrl constantPins[1];
        pinCtrl switchedPins[0];
};


typedef S2GoTemplate<pinCtrl> S2GoTemplateArduino;


#endif // S2GO_TEMPLATE_ARDUINO_HPP
