#ifndef TLX493D_KIT2GO_BOARD_SUPPORT_HPP
#define TLX493D_KIT2GO_BOARD_SUPPORT_HPP


// std include

// Arduino includes
#include <Arduino.h>

// project cpp includes
#include "Logger.h"


class Kit2GoBoardSupport {

    public:

        Kit2GoBoardSupport() : powerPins{}, selectPins{} {
        }


        void init(bool enablePower = true) {
            for(auto &p : powerPins) {
                initPin(p, enablePower);
            }

            for(auto &p : selectPins) {
                initPin(p, false);
            }
        }


        void begin(bool enablePower = true) {
            init(enablePower);
        }


        void deinit() {
            for(auto &p : powerPins) {
                controlPin(p, false);
            }

            for(auto &p : selectPins) {
                controlPin(p, false);
            }
        }


        void end() {
            deinit();
        }


        void setPowerPin(uint8_t pinNumber, uint8_t pinDirection, uint8_t pinEnableValue, uint8_t pinDisableValue,
                         uint8_t delayAfterDisable = 0, uint8_t  delayAfterEnable = 0) {
            powerPins[0] = { true, pinNumber, pinDirection, pinEnableValue, pinDisableValue, delayAfterDisable, delayAfterEnable };
        }


        void unsetPowerPin() {
            powerPins[0].isSet = false;
        }


        void setSelectPin(uint8_t pinNumber, uint8_t pinDirection, uint8_t pinEnableValue, uint8_t pinDisableValue,
                          uint8_t delayAfterDisable = 0, uint8_t  delayAfterEnable = 0) {
            selectPins[0] = { true, pinNumber, pinDirection, pinEnableValue, pinDisableValue, delayAfterDisable, delayAfterEnable };
        }


        void unsetSelectPin() {
            selectPins[0].isSet = false;
        }



        void controlPower(bool enable) {
            for(auto &p : powerPins) {
                controlPin(p, enable);
            }
        }


        void controlSelect(bool enable) {
            for(auto &p : selectPins) {
                controlPin(p, enable);
            }
        }


    private:

        typedef struct pinCtrl {
            bool     isSet;
            uint8_t  pinNumber;
            uint8_t  direction;
            uint8_t  enableValue;
            uint8_t  disableValue;
            uint8_t  delayAfterDisable;
            uint8_t  delayAfterEnable;
        } pinCtrl;


        void controlPin(const pinCtrl &p, bool enable) {
            if( p.isSet ) {
                digitalWrite(p.pinNumber, enable ? p.enableValue : p.disableValue);
                // print("Setting select pin %d to %d\n", p.pinNumber, enable ? p.enableValue : p.disableValue);
                delay(enable ? p.delayAfterEnable : p.delayAfterDisable);
            }
        }


        void initPin(const pinCtrl &p, bool enable) {
            pinMode(p.pinNumber, p.direction);
            controlPin(p, enable);
        }
    

        pinCtrl powerPins[1];
        pinCtrl selectPins[1];
};


#endif // TLX493D_KIT2GO_BOARD_SUPPORT_HPP
