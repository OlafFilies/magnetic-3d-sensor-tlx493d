#ifndef TLX493D_HPP
#define TLX493D_HPP


// std includes
#include <cstdbool>
#include <cstdint>

// project cpp includes
#include "types.hpp"
#include "BoardSupportUsingKit2Go.hpp"
#include "IICUsingTwoWire.hpp"
#include "SPIUsingSPIClass.hpp"
#include "TLx493DBase.hpp"

// project c includes
#include "tlx493d_types.h"
#include "tlx493d.h"


namespace ifx {
    namespace tlx493d {
        /**
         * @brief Specialization of the template class `TLx493D` for the IIC interface
         * 
         * @tparam BoardSupport A helper class, which takes care of needed power pins as well user defined pins.
         * @tparam TwoWireWrapper A wrapper for the I2C-class.
         * @tparam sensorType   Specific sensor type of the used sensor.
         */
        template<typename BoardSupport, TLx493D_SupportedSensorType_t sensorType>
            class TLx493D<BoardSupport, TwoWireWrapper, sensorType> : public TLx493DBase {

            public:

                typedef BoardSupport                                    BoardSupportType;
                // typedef ifx::tlx493d::TwoWireWrapper                    BusWrapperType;
                typedef typename ifx::tlx493d::TwoWireWrapper::BusType  BusType;


                /**
                 * @brief 
                */
                explicit TLx493D(BusType &busObj, TLx493D_IICAddressType_t iicAdr = TLx493D_IIC_ADDR_A0_e) : bsc(), bus(busObj), iicAddress(iicAdr) {
                // explicit TLx493D(BusType &bus, TLx493D_IICAddressType_t iicAdr = TLx493D_IIC_ADDR_A0_e) : bsc(), busWrapper(bus), iicAddress(iicAdr) {
                    (void) tlx493d_init(&sensor, sensorType);
                }

                /**
                 * @brief The function `~TLx493D` is the destructor of the template class.
                 * 
                 */
                ~TLx493D() {
                }

                /**
                 * @brief The function `init` initializes all necessary peripherals of the chosen sensor board.
                 * It initializes the BoardSupportClass, the communication interface and calls the `setDefaultConfig`
                 * function in order to disable the interrupt of the sensor and putting it into 1-Byte-Read-Mode.
                 * 
                 * @param[in] enablePower           Tells the BoardSupportClass, whether it should turn the power pins on
                 * or keep them off at initialization. 
                 * @param[in] enableSelect          Tells the BoardSupportClass, whether it should turn the select pins on
                 * or keep them off at initialization.
                 * @param[in] enableExtendedAdress    Enable or disable the extended address feature of the sensor.
                 */
                void init(bool enablePower = true, bool enableSelect = false, bool enableExtendedAdress = false) {
                    (void) initBoardSupport(&sensor, bsc);
                    bsc.init(enablePower, enableSelect, enableExtendedAdress);
                    initCommunication(&sensor, bus, iicAddress, false);
                    // initCommunication(&sensor, busWrapper, iicAddress, false);
                    setDefaultConfig();
                }

                /**
                 * @brief The function `begin` calls the `init` function with the corresponding parameters.
                 * Detailed description see `init`function.
                 * 
                 * @param[in] enablePower           Tells the BoardSupportClass, whether it should turn the power pins on
                 * or keep them off at initialization. True - by default.
                 * @param[in] enableSelect          Tells the BoardSupportClass, whether it should turn the select pins on
                 * or keep them off at initialization. False - by default.
                 * @param[in] enableExtendedAdress    Enable or disable the extended address feature of the sensor.
                 * False - by default.
                 */
                void begin(bool enablePower = true, bool enableSelect = false, bool enableExtendedAdress = false) {
                    init(enablePower, enableSelect, enableExtendedAdress);
                }

                /**
                 * @brief The function `deinit` de-initializes the sensor and its peripherals.
                 * It de-initializes the communication interface, the BoardSupportClass as well as
                 * all allocated memory (free the memory to be precise).
                 */
                void deinit() {
                    deinitCommunication(&sensor, false);
                    bsc.deinit();
                    (void) tlx493d_deinit(&sensor);
                }

                /**
                 * @brief The function `end` calls the `deinit` function.
                 * For details take a look at the `deinit` function.
                 * 
                 */
                void end() {
                    deinit();
                }

                /**
                 * @brief The function `setPowerPin` defines a power pin, which can be used to power up the sensor.
                 * 
                 * @param[in] pinNumber             Pin number of the desired pin.
                 * @param[in] pinDirection          Pin direction of the pin (OUTPUT, INPUT, INPUT-PULLUP).
                 * @param[in] pinEnableValue        Value to enable the power pin, depends on the pin (Active high or low).
                 * @param[in] pinDisableValue       Value to disable the power pin, depends on the pin (Active high or low).
                 * @param[in] delayAfterEnable      Delay after enabling the power pin in [ms].
                 * @param[in] delayAfterDisable     Delay after disabling the power pin in [ms].
                 */
                void setPowerPin(uint8_t pinNumber, uint8_t pinDriveDirection, uint8_t pinTristateDirection,
                                 uint8_t pinEnableValue, uint8_t pinDisableValue,
                                 uint32_t delayAfterEnable = 0, uint32_t delayAfterDisable = 0) {
                    bsc.setPowerPin(pinNumber, pinDriveDirection, pinTristateDirection, pinEnableValue, pinDisableValue, delayAfterEnable, delayAfterDisable);
                }

                /**
                 * @brief The function `unsetPowerPin` does remove the power pin from the BoardSupportClass.
                 * 
                 */
                void unsetPowerPin() {
                    bsc.unsetPowerPin();
                }

                /**
                 * @brief The function `setSelectPin` defines a select pin, which can be used as chip select for
                 * the SPI communication.
                 * 
                 * @param[in] pinNumber             Pin number of the desired pin.
                 * @param[in] pinDirection          Pin direction of the pin (OUTPUT, INPUT, INPUT-PULLUP).
                 * @param[in] pinEnableValue        Value to enable the power pin, depends on the pin (Active high or low).
                 * @param[in] pinDisableValue       Value to disable the power pin, depends on the pin (Active high or low).
                 * @param[in] delayAfterEnable      Delay after enabling the power pin in [ms]. 
                 * @param[in] delayAfterDisable     Delay after disabling the power pin in [ms].
                 */
                void setSelectPin(uint8_t pinNumber, uint8_t pinDriveDirection, uint8_t pinTristateDirection,
                                  uint8_t pinEnableValue, uint8_t pinDisableValue,
                                  uint32_t delayAfterEnable = 0, uint32_t delayAfterDisable = 0) {
                    bsc.setSelectPin(pinNumber, pinDriveDirection, pinTristateDirection, pinEnableValue, pinDisableValue, delayAfterEnable, delayAfterDisable);
                }

                /**
                 * @brief The function `unsetSelectPin` does remove the select pin from the BoardSupportClass.
                 * 
                 */
                void unsetSelectPin() {
                    bsc.unsetSelectPin();
                }

                /**
                 * @brief The function `setAdressPin` defines a pin, which can be used for the extended address feature of the first
                 * sensor generation. Here the SDA-line of the I2C bus has to be pulled low in order to use
                 * the second half of addresses.
                 * 
                 * @param[in] pinNumber             Pin number of the desired pin.
                 * @param[in] pinDirection          Pin direction of the pin (OUTPUT, INPUT, INPUT-PULLUP).
                 * @param[in] pinEnableValue        Value to enable the power pin, depends on the pin (Active high or low).
                 * @param[in] pinDisableValue       Value to disable the power pin, depends on the pin (Active high or low).
                 * @param[in] delayAfterEnable      Delay after enabling the power pin in [ms]. 
                 * @param[in] delayAfterDisable     Delay after disabling the power pin in [ms].
                 */
                void setAddressPin(uint8_t pinNumber, uint8_t pinDriveDirection, uint8_t pinTristateDirection,
                                   uint8_t pinEnableValue, uint8_t pinDisableValue,
                                   uint32_t delayAfterEnable = 0, uint32_t delayAfterDisable = 0) {
                    bsc.setAdressPin(pinNumber, pinDriveDirection, pinTristateDirection, pinEnableValue, pinDisableValue, delayAfterEnable, delayAfterDisable);
                }

                /**
                 * @brief The function `unsetAdressPin` does remove the address pin from the BoardSupportClass.
                 * 
                 */
                void unsetAddressPin() {
                    bsc.unsetAdressPin();
                }

                /**
                 * @brief The function `enablePower` enables the defined power pin.
                 * 
                 */
                void enablePower() {
                    bsc.enablePower(true);
                }

                /**
                 * @brief The function `disablePower` disables the defined power pin.
                 * 
                 */
                void disablePower() {
                    bsc.enablePower(false);
                }

                void reset() {
                    deinitCommunication(&sensor, true);
                    sensor.functions->setResetValues(&sensor);
                    bsc.reset();
                    initCommunication(&sensor, bus, iicAddress, true);
                    // initCommunication(&sensor, busWrapper, iicAddress, true);
                    setDefaultConfig();
                }

                /**
                 * @brief The function `enableSelect` enables the defined select pin.
                 * 
                 */
                void enableSelect() {
                    bsc.enableSelect(true);
                }

                /**
                 * @brief The function `disableSelect` disables the defined select pin.
                 * 
                 */
                void disableSelect() {
                    bsc.enableSelect(false);
                }

                /**
                 * @brief The function `enableAdress` enables the defined address pin.
                 * 
                 */
                void enableAddress() {
                    bsc.enableAddress(true);
                }

                /**
                 * @brief The function `disableAdress` disables the defined address pin.
                 * 
                 */
                void disableAddress() {
                    bsc.enableAddress(false);
                }


            private:

                /**
                 * @brief The function `TLx493D` is the overloaded constructor of the template class.
                 * 
                 * @param[in] bus Bus type of the chose sensor board.
                 */
                TLx493D(BusType &bus);


                BoardSupportType          bsc;          /**< BoardSupportClass */
                // BusWrapperType            busWrapper;   /**< BusWrapperClass */
                BusType                   bus;  /**< Bus Class */
                TLx493D_IICAddressType_t  iicAddress;   /**< I2C-Address of the sensor */
        };


        /**
         * @brief Specialization of the template class `TLx493D` for the SPI interface
         * 
         * @tparam BoardSupport A helper class, which takes care of needed power pins as well user defined pins.
         * @tparam sensorType   Specific sensor type of the used sensor.
         */
        template<typename BoardSupport, TLx493D_SupportedSensorType_t sensorType>
            class TLx493D<BoardSupport, SPIClassWrapper, sensorType> : public TLx493DBase {

            public:

                typedef BoardSupport                                     BoardSupportType;
                // typedef ifx::tlx493d::SPIClassWrapper                    BusWrapperType;
                typedef typename ifx::tlx493d::SPIClassWrapper::BusType  BusType;

        
                explicit TLx493D(BusType &busObj) : bsc(), bus(busObj) {
                // explicit TLx493D(BusType &bus) : bsc(), busWrapper(bus) {
                    (void) tlx493d_init(&sensor, sensorType);
                }

                /**
                 * @brief The function `~TLx493D` is the constructor of the template class.
                 * 
                 */
                ~TLx493D() {
                }

                /**
                 * @brief The function `init` initializes all necessary peripherals of the chosen sensor board.
                 * It initializes the BoardSupportClass, the communication interface and calls the `setDefaultConfig`
                 * function
                 *
                 */
                void init(bool enablePower = false, bool enableSelect = true) {
                    (void) initBoardSupport(&sensor, bsc);
                    bsc.init(enablePower, enableSelect, false);
                    // initCommunication(&sensor, busWrapper, true);
                    initCommunication(&sensor, bus, true);
                    setDefaultConfig();
                }

                /**
                 * @brief The function `begin` calls the `init` function.
                 * For details see `init` function.
                 * 
                 */
                void begin(bool enablePower = false, bool enableSelect = true) {
                    init(enablePower, enableSelect);
                }


                /**
                 * @brief The function `deinit` de-initializes the sensor and its peripherals.
                 * It de-initializes the communication interface, the BoardSupportClass as well as
                 * all allocated memory (free the memory to be precise).
                 * 
                 */
                void deinit() {
                    deinitCommunication(&sensor, false);
                    bsc.deinit();
                    (void) tlx493d_deinit(&sensor);
                }

                /**
                 * @brief The function `end` calls the `deinit` function.
                 * For details see `deinit`function.
                 * 
                 */
                void end() {
                    deinit();
                }

                /**
                 * @brief The function `setPowerPin` defines a power pin, which can be used to power up the sensor.
                 * 
                 * @param[in] pinNumber             Pin number of the desired pin.
                 * @param[in] pinDirection          Pin direction of the pin (OUTPUT, INPUT, INPUT-PULLUP).
                 * @param[in] pinEnableValue        Value to enable the power pin, depends on the pin (Active high or low).
                 * @param[in] pinDisableValue       Value to disable the power pin, depends on the pin (Active high or low).
                 * @param[in] delayAfterEnable      Delay after enabling the power pin in [ms].
                 * @param[in] delayAfterDisable     Delay after disabling the power pin in [ms].
                 */
                void setPowerPin(uint8_t pinNumber, uint8_t pinDriveDirection, uint8_t pinTristateDirection,
                                 uint8_t pinEnableValue, uint8_t pinDisableValue,
                                 uint32_t delayAfterEnable = 0, uint32_t delayAfterDisable = 0) {
                    bsc.setPowerPin(pinNumber, pinDriveDirection, pinTristateDirection, pinEnableValue, pinDisableValue, delayAfterEnable, delayAfterDisable);
                }

                /**
                 * @brief The function `unsetPowerPin` does remove the power pin from the BoardSupportClass.
                 * 
                 */
                void unsetPowerPin() {
                    bsc.unsetPowerPin();
                }

                /**
                 * @brief The function `setSelectPin` defines a select pin, which can be used as chip select for
                 * the SPI communication.
                 * 
                 * @param[in] pinNumber             Pin number of the desired pin.
                 * @param[in] pinDirection          Pin direction of the pin (OUTPUT, INPUT, INPUT-PULLUP).
                 * @param[in] pinEnableValue        Value to enable the power pin, depends on the pin (Active high or low).
                 * @param[in] pinDisableValue       Value to disable the power pin, depends on the pin (Active high or low).
                 * @param[in] delayAfterEnable      Delay after enabling the power pin in [ms]. 
                 * @param[in] delayAfterDisable     Delay after disabling the power pin in [ms].
                 */
                void setSelectPin(uint8_t pinNumber, uint8_t pinDriveDirection, uint8_t pinTristateDirection,
                                  uint8_t pinEnableValue, uint8_t pinDisableValue,
                                  uint32_t delayAfterEnable = 0, uint32_t delayAfterDisable = 0) {
                    bsc.setSelectPin(pinNumber, pinDriveDirection, pinTristateDirection, pinEnableValue, pinDisableValue, delayAfterEnable, delayAfterDisable);
                }

                /**
                 * @brief The function `unsetSelectPin` does remove the select pin from the BoardSupportClass.
                 * 
                 */
                void unsetSelectPin() {
                    bsc.unsetSelectPin();
                }

                /**
                 * @brief The function `enablePower` enables the defined power pin.
                 * 
                 */
                void enablePower() {
                    bsc.enablePower(true);
                }

                /**
                 * @brief The function `disablePower` disables the defined power pin.
                 * 
                 */
                void disablePower() {
                    bsc.enablePower(false);
                }

                /**
                 * @brief The function `reset` set the sensor's registers to its reset values and resets the
                 * communication interface. After that the register maps will be set to its default values.
                 */
                void reset() {
                    deinitCommunication(&sensor, false);
                    sensor.functions->setResetValues(&sensor);
                    bsc.reset();
                    // initCommunication(&sensor, busWrapper, true);
                    initCommunication(&sensor, bus, true);
                    setDefaultConfig();
                }

                /**
                 * @brief The function `enableSelect` enables the defined select pin.
                 * 
                 */
                void enableSelect() {
                    bsc.enableSelect(true);
                }

                /**
                 * @brief The function `disableSelect` disables the defined select pin.
                 * 
                 */
                void disableSelect() {
                    bsc.enableSelect(false);
                }


            private:

                BoardSupportType  bsc;              /**< BoardSupportClass */
                // BusWrapperType    busWrapper;    /**< BusWrapperClass */
                BusType           bus;              /**< Bus Class */

        };
    }
}


#endif // TLX493D_HPP
