#ifndef TLX493D_SPICLASS_WRAPPER_HPP
#define TLX493D_SPICLASS_WRAPPER_HPP


// std includes

// Arduino includes
#include <Arduino.h>
#include <SPI.h>


// SPI
#define GEN_3_SPI_READ_WRITE_BIT        0x80
#define GEN_3_SPI_READ_BIT_ON           0x80
#define GEN_3_SPI_WRITE_BIT_OFF         0x00

#define GEN_3_SPI_AUTO_INC_BIT          0x60
#define GEN_3_SPI_AUTO_INC_BIT_ON       0x60
#define GEN_3_SPI_AUTO_INC_BIT_OFF      0x00


class SPIClassWrapper {

    public:

        SPIClassWrapper(SPIClass &bus) : spi(bus) {
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
                spi.transfer(GEN_3_SPI_READ_BIT_ON);

                for(; bytesRead < rxLen; ++bytesRead) {
                    rxBuffer[bytesRead] = spi.transfer(GEN_3_SPI_READ_BIT_ON);
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


#endif // TLX493D_SPICLASS_WRAPPER_HPP
