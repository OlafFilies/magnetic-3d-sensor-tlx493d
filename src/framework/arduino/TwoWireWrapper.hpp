#ifndef TLX493D_TWOWIRE_WRAPPER_HPP
#define TLX493D_TWOWIRE_WRAPPER_HPP


// std includes

// Arduino includes
#include <Arduino.h>
#include <Wire.h>


class TwoWireWrapper {

    public:

        typedef TwoWire BusType;

        explicit TwoWireWrapper(TwoWire &bus) : iic(bus) {
        }


        void init() {
            iic.begin();
        }


        void deinit() {
            iic.end();
        }


        bool transfer(uint8_t iicAddress, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen) {
            if( (txLen > 0) && (txBuffer != NULL) ) {
                iic.beginTransmission(iicAddress);

                uint8_t bytesWritten = iic.write(txBuffer, txLen);
                iic.endTransmission(true);

                if( bytesWritten != txLen ) {
                    return false;
                }
            }

            if( (rxLen > 0)  && (rxBuffer != NULL) ) {
                uint8_t bytesRead = iic.requestFrom(iicAddress, rxLen);

                for(uint16_t i = 0; (i < rxLen) && (iic.available() > 0); ++i) {
                    rxBuffer[i] = iic.read();
                }

                // iic.endTransmission(true);

                if( bytesRead != rxLen ) {
                    return false;
                }
            }

            return true;
        }


        TwoWire &getBus() {
            return iic;
        }


    private:

      TwoWire &iic;
};


#endif // TLX493D_TWOWIRE_WRAPPER_HPP
