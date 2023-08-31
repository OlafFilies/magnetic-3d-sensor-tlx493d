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


        bool transfer(uint8_t i2cAddress, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len) {
            // TODO: get channel from used Wire object !
           	//  XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, 0xFFFFFFFF);

            if( tx_len > 0 && tx_buffer != NULL ) {
                i2c.beginTransmission(i2cAddress);

                uint8_t written = i2c.write(tx_buffer, tx_len);
                i2c.endTransmission(true);

                if( written != tx_len ) {
                    return false;
                }
            }

            if( rx_len > 0  && rx_buffer != NULL ) {
                uint8_t bytes_read = i2c.requestFrom(i2cAddress, rx_len);

                for(uint16_t i = 0; (i < rx_len) && (i2c.available() > 0); ++i) {
                    rx_buffer[i] = i2c.read();
                }

                i2c.endTransmission(true);

                if( bytes_read != rx_len ) {
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
