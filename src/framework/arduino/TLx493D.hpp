#ifndef TLX493D_HPP
#define TLX493D_HPP


// std includes
#include <stdbool.h>
#include <stdint.h>

// project cpp includes
#include "types.hpp"
#include "IICUsingTwoWire.hpp"
#include "SPIUsingSPIClass.hpp"
#include "TLx493DBase.hpp"

// project c includes
#include "tlx493d_types.h"
#include "tlx493d.h"


/***
 * Specialization for IIC interface.
*/
template<typename BoardSupport,
         TLx493D_SupportedSensorType_t sensorType> class TLx493D<BoardSupport, TwoWireWrapper,
                                                                 TwoWire, sensorType> : public TLx493DBase {

    public:

        typedef BoardSupport    BoardSupportType;
        typedef TwoWireWrapper  BusWrapperType;

 
        TLx493D(TwoWire &bus, TLx493D_IICAddressType_t iicAdr = TLx493D_IIC_ADDR_A0_e) : bsc(), busWrapper(bus), iicAddress(iicAdr) {
            ::tlx493d_init(&sensor, sensorType);
        }


        ~TLx493D() {
        }


        void init() {
            bsc.init();
            tlx493d_initCommunication(&sensor, busWrapper, iicAddress); // includes call to busWrapper.init();
            setDefaultConfig();
        }


        void begin() {
            init();
        }


        void deinit() {
            ::tlx493d_deinit(&sensor);
            busWrapper.deinit();
            bsc.deinit();
        }


        void end() {
            deinit();
        }


    private:

        TLx493D(TwoWire &bus);


        BoardSupportType          bsc;
        BusWrapperType            busWrapper;
        TLx493D_IICAddressType_t  iicAddress;
};


/***
 * Specialization for SPI interface.
*/
template<typename BoardSupport,
         TLx493D_SupportedSensorType_t sensorType> class TLx493D<BoardSupport, SPIClassWrapper,
                                                                 SPIClass, sensorType> : public TLx493DBase {

    public:

        typedef BoardSupport     BoardSupportType;
        typedef SPIClassWrapper  BusWrapperType;

 
        TLx493D(SPIClass &bus) : bsc(), busWrapper(bus) {
            ::tlx493d_init(&sensor, sensorType);
        }


        ~TLx493D() {
        }


        void init() {
            bsc.init();
            tlx493d_initCommunication(&sensor, busWrapper); // includes call to busWrapper.init();
            setDefaultConfig();
        }


        void begin() {
            init();
        }


        void deinit() {
            ::tlx493d_deinit(&sensor);
            busWrapper.deinit();
            bsc.deinit();
        }


        void end() {
            deinit();
        }


    private:

        BoardSupportType  bsc;
        BusWrapperType    busWrapper;
};


#endif // TLX493D_HPP
