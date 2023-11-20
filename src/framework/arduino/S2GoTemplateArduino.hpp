
#ifndef TLX493D_S2GO_TEMPLATE_ARDUINO_HPP
#define TLX493D_S2GO_TEMPLATE_ARDUINO_HPP


// std include

// Arduino includes
#include <Arduino.h>

// project cpp includes
#include "S2GoTemplate.hpp"


typedef struct pinCtrl {
    uint8_t pinNumber;
    uint8_t direction;
    uint8_t enableValue;
    uint8_t disableValue;
} pinCtrl;


template<> class S2GoTemplate<pinCtrl> {
   public:

        S2GoTemplate() : constantPins{ LED2, OUTPUT, HIGH, LOW}, switchedPins{} {
        }

        void init() {
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
            for(auto &p : switchedPins) {
                digitalWrite(p.pinNumber, p.enableValue);
            }
        }


        void disable() {
            for(auto &p : switchedPins) {
                digitalWrite(p.pinNumber, p.disableValue);
            }
        }


    private:

        pinCtrl constantPins[1];
        pinCtrl switchedPins[0];
};


#endif // TLX493D_S2GO_TEMPLATE_ARDUINO_HPP
