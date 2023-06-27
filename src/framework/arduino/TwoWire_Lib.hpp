#ifndef TWOWIRE_LIB_HPP
#define TWOWIRE_LIB_HPP


// std includes

// Arduino includes
#include <Arduino.h>
#include <Wire.h>

// project cpp includes
#include "arduino_defines.h"

// project c includes


template<typename ComIF> class TwoWire_Lib {
};


template<> class TwoWire_Lib<TwoWire> {
   public:

        TwoWire_Lib() : i2c(Wire), i2cAddress() {
        }


        TwoWire_Lib(TwoWire &comif) : i2c(comif), i2cAddress() {
        }


        void init(Sensor_ts *sensor) {
            i2cAddress = sensor->comLibIFParams.i2c_params.address;
            // TODO: reset sensor first
           i2c.begin();
        }


        void deinit() {
            i2c.end();
        }


        bool transfer(uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len) {
            if( tx_len > 0 ) {
                i2c.beginTransmission(i2cAddress);

                uint8_t written = i2c.write(tx_buffer, tx_len);
    // log("i2c tx_len : "); log(tx_len); log("\n");
    // log("i2c written : "); log(written); log("\n");

                i2c.endTransmission(true);

                if( written != tx_len ) {
                    return false;
                }
            }

            if( rx_len > 0 ) {
                uint8_t bytes_read = i2c.requestFrom(i2cAddress, rx_len);
// log("i2c bytes_read : "); log(bytes_read); log("\n");
// log("i2c rx_len : "); log(rx_len); log("\n");

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


    private:

      TwoWire &i2c;
      uint8_t  i2cAddress;
};


#endif // TWOWIRE_LIB_HPP
