#ifndef TWOWIRE_LIB_HPP
#define TWOWIRE_LIB_HPP


// std includes

// Arduino includes
#include <Arduino.h>
#include <Wire.h>

// project cpp includes
#include "arduino_defines.h"


template<typename ComIF> class TwoWire_Lib {
};


template<> class TwoWire_Lib<TwoWire> {
   public:


        TwoWire_Lib(TwoWire &comif) : i2c(comif) {
        }


        void init() {
            i2c.begin();
        }


        void deinit() {
            i2c.end();
        }


        bool transfer(uint8_t i2cAddress, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen) {
            // TODO: get channel from used Wire object !
           	//  XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, 0xFFFFFFFF);

            if( (txLen > 0) && (txBuffer != NULL) ) {
                i2c.beginTransmission(i2cAddress);

                uint8_t bytesWritten = i2c.write(txBuffer, txLen);
                i2c.endTransmission(true);

                if( bytesWritten != txLen ) {
                    return false;
                }
            }

            if( (rxLen > 0)  && (rxBuffer != NULL) ) {
                uint8_t bytesRead = i2c.requestFrom(i2cAddress, rxLen);

                for(uint16_t i = 0; (i < rxLen) && (i2c.available() > 0); ++i) {
                    rxBuffer[i] = i2c.read();
                }

                i2c.endTransmission(true);

                if( bytesRead != rxLen ) {
                    return false;
                }
            }

            return true;
        }


        TwoWire &getComIF() {
            return i2c;
        }


    private:

      TwoWire &i2c;
};


#endif // TWOWIRE_LIB_HPP
