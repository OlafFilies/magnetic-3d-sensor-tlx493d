#ifndef SPI_LIB_HPP
#define SPI_LIB_HPP


// std includes

// Arduino includes
#include <Arduino.h>
#include <SPI.h>

// project cpp includes
#include "arduino_defines.h"

#include "Logger.h"

template<typename ComIF> class SPI_Lib {
};


template<> class SPI_Lib<SPIClass> {
   public:

        SPI_Lib(SPIClass &comif) : spi(comif) {
        }


        void init() {
            spi.begin();
        }


        void deinit() {
            spi.end();
        }


        bool transfer(uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen) {
             if( (txLen > 0) && (txBuffer != NULL) ) {
                uint8_t bytesWritten = 0;
                    // logmsgui("writing : ", (uint8_t) txLen);

                for(; bytesWritten < txLen; ++bytesWritten) {
                    // logmsgui("writing : ", txBuffer[bytesWritten]);
                    spi.transfer(txBuffer[bytesWritten]);
                }

                if( bytesWritten != txLen ) {
                    return false;
                }
            }

            if( (rxLen > 0)  && (rxBuffer != NULL) ) {
                uint16_t bytesRead = 0;
                    // logmsgui("reading from : ", (uint8_t) 0x80);
                    rxBuffer[bytesRead] = spi.transfer(0x80);

                for(; bytesRead < rxLen; ++bytesRead) {
                    rxBuffer[bytesRead] = spi.transfer(0x80);
                    // logmsgui("read : ", rxBuffer[bytesRead]);
                }

                if( bytesRead != rxLen ) {
                    return false;
                }
            }

            return true;
        }


        SPIClass &getComIF() {
            return spi;
        }


    private:

      SPIClass &spi;
};


#endif // SPI_LIB_HPP
