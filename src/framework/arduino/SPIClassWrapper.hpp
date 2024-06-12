#ifndef TLX493D_SPICLASS_WRAPPER_HPP
#define TLX493D_SPICLASS_WRAPPER_HPP


// std includes

// Arduino includes
#include <Arduino.h>
#include <SPI.h>


namespace ifx {
    namespace tlx493d {
        /**
         * @brief The class `SPIClassWrapper` is used to make the generic `SPIClass` compatible with Arduino.
         * It wraps the required functions into the Arduino API calls.
         */
        class SPIClassWrapper {

            public:

                /** Sets the bus type to SPI. */
                using BusType = SPIClass;

                // static constexpr uint8_t TLX493D_SPI_READ_BIT         = 0x80;
                static constexpr uint8_t TLX493D_SPI_READ_BIT_ON      = 0x80;
                // static constexpr uint8_t TLX493D_SPI_READ_BIT_OFF     = 0x00;

                // static constexpr uint8_t TLX493D_SPI_AUTO_INC_BIT     = 0x60;
                // static constexpr uint8_t TLX493D_SPI_AUTO_INC_BIT_ON  = 0x60;
                // static constexpr uint8_t TLX493D_SPI_AUTO_INC_BIT_OFF = 0x00;

                /**
                 * @brief Constructor of the `SPIClassWrapper`class with no parameters.
                 */
                explicit SPIClassWrapper() : spi(nullptr) {
                }

                /**
                 * @brief Constructor of the `SPIClassWrapper`class with a `SPIClass` parameter. Can
                 * be used to pass a desired SPI instance to the wrapper.
                 */
                explicit SPIClassWrapper(SPIClass &bus) : spi(&bus) {
                }

                /**
                 * @brief Destructor of the `SPIClassWrapper`.
                 */
                ~SPIClassWrapper() {
                }


                /**
                 * @brief The SPIClass::init does not include the setting of the data mode, bit order and baudrate, all of which
                 * is done in SPIClass::beginTransaction(SPISettings ). But we do not know about such details at this level. 
                 * Therefore init() is not sufficient to restart a SPIClass object after a reset !
                 * 
                 * @param[in] settings Settings for the SPI communication interface.
                 */
                void init(const SPISettings &settings) {
                // void init(uint32_t clockFreq, uint8_t bitOrder, uint8_t dataMode) {
                    // SPISettings settings(clockFreq, bitOrder, dataMode);
                    spi->beginTransaction(settings);
                    // spi.beginTransaction(settings);

                    // spi.begin();
                }

                /**
                 * @brief The function `deinit` de-initializes the SPI interface. 
                 */
                void deinit() {
                    spi->end();
                    // spi.end();
                }

                /**
                 * @brief The function `transfer` is used to transfer data.
                 * 
                 * @param[in] txBuffer Transmit buffer.
                 * @param[in] txLen Length of the data that should be transmitted.
                 * @param[in] rxBuffer Receive buffer.
                 * @param[in] rxLen Length of the data that should be received.
                 * @param[in] readAddress Desired address where the device should read from.
                 * 
                 * @return Function returns a boolean value to indicate if the transfer was successful or not.
                 * @retval 0 Error.
                 * @retval 1 Success.
                 */
                bool transfer(uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen, uint8_t readAddress) {
                    if( (txLen > 0) && (txBuffer != NULL) ) {
                        uint8_t bytesWritten = 0;

                        for(; bytesWritten < txLen; ++bytesWritten) {
                            spi->transfer(txBuffer[bytesWritten]);
                            // spi.transfer(txBuffer[bytesWritten]);
                        }

                        if( bytesWritten != txLen ) {
                            return false;
                        }
                    }

                    if( (rxLen > 0)  && (rxBuffer != NULL) ) {
                        uint16_t bytesRead = 0;
                        spi->transfer(TLX493D_SPI_READ_BIT_ON | readAddress);
                        // spi.transfer(TLX493D_SPI_READ_BIT_ON | readAddress);

                        for(; bytesRead < rxLen; ++bytesRead) {
                            rxBuffer[bytesRead] = spi->transfer(TLX493D_SPI_READ_BIT_ON | readAddress);
                            // rxBuffer[bytesRead] = spi.transfer(TLX493D_SPI_READ_BIT_ON | readAddress);
                        }

                        if( bytesRead != rxLen ) {
                            return false;
                        }
                    }

                    return true;
                }

                /**
                 * @brief The function `getBus` is used to retrieve a pointer to the wrapper's bus-object.
                 * 
                 * @return A pointer to the current SPI bus-object of the wrapper.
                 */
                SPIClass &getBus() {
                    return *spi;
                }

                /**
                 * @brief The function `setBus` is used to set the `SPIClass` object of the wrapper.
                 * 
                 * @param[in] spiObj Reference to the desired `SPIClass` object. 
                 */
                void setBus(SPIClass &spiObj) {
                    spi = &spiObj;
                }


            private:
                /** SPIClass object. */
                SPIClass *spi;
        };
    }
}

#endif // TLX493D_SPICLASS_WRAPPER_HPP
