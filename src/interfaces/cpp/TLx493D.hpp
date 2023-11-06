#ifndef TLx493D_HPP
#define TLx493D_HPP

// std includes
#include <stdbool.h>
#include <stdint.h>

// project cpp includes
#include "arduino_defines.h"

// project c includes
extern "C" {
    #include "tlx493d_types.h"
    #include "cInterface.h"
}


// keyword "export" not supported by current Arduino C++ compiler, therefore definitions must go here. Update C++ compiler to newer version ???
// export
template<typename BoardSupportClass, template<typename> typename ComLibrary, typename ComIF,
         TLx493D_SupportedSensorType_te sensorType,
         TLx493D_SupportedComLibraryInterfaceType_te comLibIFType = TLx493D_I2C_e> class TLx493D {
    public:
        typedef BoardSupportClass   BoardSupportClassType;
        typedef ComLibrary<ComIF>   ComLibraryIFType;
        typedef ComIF               ComIFType;

 
        TLx493D(ComIF &comIF) : bsc(), comLIF(comIF), sensor() {
            ::tlx493d_init(&sensor, sensorType);
            sensor.comIFType = comLibIFType;
        }


        ~TLx493D() {
        }


        void init() {
            bsc.init();
            TLx493D_initCommunication(&sensor, comLIF); // includes call to comLIF.init();
            setDefaultConfig();
        }


        void begin() {
            init();
        }


        void deinit() {
            ::tlx493d_deinit(&sensor);
            comLIF.deinit();
            bsc.deinit();
        }


        void end() {
            deinit();
        }


        bool readRegisters() {
            return ::tlx493d_readRegisters(&sensor);
        }


        bool getTemperature(double *temperature) {
            return ::tlx493d_getTemperature(&sensor, temperature);
        }


        bool getMagneticField(double *x, double *y, double *z) {
            return ::tlx493d_getMagneticField(&sensor, x, y, z);
        }


        bool getMagneticFieldAndTemperature(double *x, double *y, double *z, double *temp) {
            return ::tlx493d_getMagneticFieldAndTemperature(&sensor, x, y, z, temp);
        }


        // functions related to the "Config" register
        bool tlx493d_selectMeasureValues(TLx493D_MeasureType_te meas) {
            return ::tlx493d_selectMeasureValues(&sensor, meas);
        }

        // bool enableTemperatureMeasurement() {
        //     return ::tlx493d_enableTemperatureMeasurement(&sensor);
        // }
        // bool disableTdisableTemperatureMeasurementemperature() {
        //     return ::tlx493d_disableTemperatureMeasurement(&sensor);
        // }    
        // bool enableAngularMeasurement() {
        //     return ::tlx493d_enableAngularMeasurement(&sensor);
        // }
        // bool disableAngularMeasurement() {
        //     return ::tlx493d_disableAngularMeasurement(&sensor);
        // }


        bool setTrigger(uint8_t bits) {
            return ::tlx493d_setTrigger(&sensor, bits);
        }

        
        bool setSensitivity(TLx493D_SensitivityType_te range) {
            return ::tlx493d_setSensitivity(&sensor, range);
        }


        // functions related to the "Mod1" and "Mod2" registers
        bool setDefaultConfig() {
            return ::tlx493d_setDefaultConfig(&sensor);
        }

        
        bool setIICAddress(uint8_t addr) {
            return ::tlx493d_setIICAddress(&sensor, addr);
        }


        bool enableInterrupt() {
            return ::tlx493d_enableInterrupt(&sensor);
        }


        bool disableInterrupt() {
            return ::tlx493d_disableInterrupt(&sensor);
        }


        bool enableCollisionAvoidance() {
            return ::tlx493d_enableCollisionAvoidance(&sensor);
        }


        bool disableCollisionAvoidance() {
            return ::tlx493d_disableCollisionAvoidance(&sensor);
        }


        bool setPowerMode(TLx493D_PowerModeType_te mode) {
            return ::tlx493d_setPowerMode(&sensor, mode);
        }

        
        bool setUpdateRate(uint8_t bit) {
            return ::tlx493d_setUpdateRate(&sensor, bit);
        }


        // functions related to the "Diag" register
        bool tlx493d_hasValidData() {
            return ::tlx493d_hasValidData(&sensor);
        }

        bool tlx493d_isFunctional() {
            return ::tlx493d_isFunctional(&sensor);
        }


        // functions available only to a subset of sensors with wake-up functionality
        bool isWakeUpActive() {
            return ::tlx493d_isWakeUpActive(&sensor);
        }


        bool enableWakeUpMode() {
            return ::tlx493d_enableWakeUpMode(&sensor);
        }


        bool disableWakeUpMode() {
            return ::tlx493d_disableWakeUpMode(&sensor);
        }
        
        bool setLowerWakeUpThresholdX(int16_t threshold) {
            return ::tlx493d_setLowerWakeUpThresholdX(&sensor, threshold);
        }

        bool setLowerWakeUpThresholdY(int16_t threshold) {
            return ::tlx493d_setLowerWakeUpThresholdY(&sensor, threshold);
        }

        bool setLowerWakeUpThresholdZ(int16_t threshold) {
            return ::tlx493d_setLowerWakeUpThresholdZ(&sensor, threshold);
        }

        bool setUpperWakeUpThresholdX(int16_t threshold) {
            return ::tlx493d_setUpperWakeUpThresholdX(&sensor, threshold);
        }

        bool setUpperWakeUpThresholdY(int16_t threshold) {
            return ::tlx493d_setUpperWakeUpThresholdY(&sensor, threshold);
        }

        bool setUpperWakeUpThresholdZ(int16_t threshold) {
            return ::tlx493d_setUpperWakeUpThresholdZ(&sensor, threshold);
        }

        bool setWakeUpThresholdsAsInteger(int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th) {
            return ::tlx493d_setWakeUpThresholdsAsInteger(&sensor, xl_th, xh_th, yl_th, yh_th, zl_th, zh_th);
        }

        bool setWakeUpThresholds(double xl_th, double xh_th, double yl_th, double yh_th, double zl_th, double zh_th) {
            return ::tlx493d_setWakeUpThresholds(&sensor, xl_th, xh_th, yl_th, yh_th, zl_th, zh_th);
        }


        // utilities
        bool tlx493d_softReset() {
            return ::tlx493d_softReset(&sensor);
        }

        const char *tlx493d_getTypeAsString() {
            return ::tlx493d_getTypeAsString(&sensor);
        }


        // Attribute getters for TLx493D object
        TLx493D_ts *getSensor() {
            return &sensor;
        }


        TLx493D_SupportedSensorType_te getSensorType() {
            return sensor.sensorType;
        }


        TLx493D_SupportedComLibraryInterfaceType_te getComLibIFType() {
            return sensor.comIFType;
        }


        uint8_t *getRegisterMap() {
            return sensor.regMap;
        }


        uint8_t getRegisterMapSize() {
            return sensor.regMapSize;
        }


        uint8_t getI2CAddress() {
            return sensor.comLibIFParams.i2c_params.address;
        }


    private:

        BoardSupportClassType  bsc;
        ComLibraryIFType       comLIF;
        TLx493D_ts             sensor;
};

#endif // TLx493D_HPP
