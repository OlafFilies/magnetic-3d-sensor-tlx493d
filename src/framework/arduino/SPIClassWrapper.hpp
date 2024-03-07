#ifndef TLX493D_SPICLASS_WRAPPER_HPP
#define TLX493D_SPICLASS_WRAPPER_HPP


// std includes

// Arduino includes
#include <Arduino.h>
#include <SPI.h>


namespace ifx {
    namespace tlx493d {
        class SPIClassWrapper {

            public:

                using BusType = SPIClass;

                // static constexpr uint8_t TLX493D_SPI_READ_BIT         = 0x80;
                static constexpr uint8_t TLX493D_SPI_READ_BIT_ON      = 0x80;
                // static constexpr uint8_t TLX493D_SPI_READ_BIT_OFF     = 0x00;

                // static constexpr uint8_t TLX493D_SPI_AUTO_INC_BIT     = 0x60;
                // static constexpr uint8_t TLX493D_SPI_AUTO_INC_BIT_ON  = 0x60;
                // static constexpr uint8_t TLX493D_SPI_AUTO_INC_BIT_OFF = 0x00;

                explicit SPIClassWrapper(SPIClass &bus) : spi(bus) {
                }


                ~SPIClassWrapper() {
                }


                /**
                 * @brief The SPIClass::init does not include the setting of the data mode, bit order and baudrate, all of which
                 * is done in SPIClass::beginTransaction(SPISettings ). But we do not know about such details at this level. 
                 * Therefore init() is not sufficient to restart a SPIClass object after a reset !
                 * 
                 */
                void init() {
                    spi.begin();
                }


                void deinit() {
                    spi.end();
                }


                bool transfer(uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen, uint8_t readAddress) {
                    if( (txLen > 0) && (txBuffer != NULL) ) {
                        uint8_t bytesWritten = 0;

                        for(; bytesWritten < txLen; ++bytesWritten) {
                            spi.transfer(txBuffer[bytesWritten]);
                        }

                        if( bytesWritten != txLen ) {
                            return false;
                        }
                    }

                    if( (rxLen > 0)  && (rxBuffer != NULL) ) {
                        uint16_t bytesRead = 0;
                        spi.transfer(TLX493D_SPI_READ_BIT_ON | readAddress);

                        for(; bytesRead < rxLen; ++bytesRead) {
                            rxBuffer[bytesRead] = spi.transfer(TLX493D_SPI_READ_BIT_ON | readAddress);
                        }

                        if( bytesRead != rxLen ) {
                            return false;
                        }
                    }

                    return true;
                }


                SPIClass &getBus() {
                    return spi;
                }


            private:

                SPIClass &spi;
        };
    }
}

#endif // TLX493D_SPICLASS_WRAPPER_HPP
