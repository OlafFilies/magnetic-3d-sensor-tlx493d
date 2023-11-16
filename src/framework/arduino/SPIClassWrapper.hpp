#ifndef TLX493D_SPI_LIB_HPP
#define TLX493D_SPI_LIB_HPP


// std includes

// Arduino includes
#include <Arduino.h>
#include <SPI.h>


template<typename Protocol> class SPIClassWrapper {
};


template<> class SPIClassWrapper<SPIClass> {
   public:

        SPIClassWrapper(SPIClass &comif) : spi(comif) {
        }


        void init() {
            spi.begin();
        }


        void deinit() {
            spi.end();
        }


        bool transfer(uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen) {
            uint8_t bytesWritten = 0;

             if( (txLen > 0) && (txBuffer != NULL) ) {
                for(; bytesWritten < txLen; ++bytesWritten) {
                    spi.transfer(txBuffer[bytesWritten]);
                }

                if( bytesWritten != txLen ) {
                    return false;
                }
            }

            if( (rxLen > 0)  && (rxBuffer != NULL) ) {
                uint16_t bytesRead = 0;
                spi.transfer(0x80);

                for(; bytesRead < rxLen; ++bytesRead) {
                    rxBuffer[bytesRead] = spi.transfer(0x80);
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


#endif // TLX493D_SPI_LIB_HPP
