#ifndef TLX493D_SPI_LIB_HPP
#define TLX493D_SPI_LIB_HPP


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


#endif // TLX493D_SPI_LIB_HPP
