#ifndef MAGNETIC_3D_HPP
#define MAGNETIC_3D_HPP

// std includes
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

// project cpp includes
#include "arduino_defines.h"

// project c includes
extern "C" {
    #include "sensor_types.h"
    #include "sensors_common.h"
}


// keyword "export" not supported by current Arduino C++ compiler, therefore definitions must go here. Update C++ compiler to newer version ???
// export
template<typename BoardSupportClass, template<typename> typename ComLibrary, typename ComIF, typename SensorClass> class Sensor3D {
    public:
        typedef BoardSupportClass               BoardSupportClassType;
        typedef ComLibrary<ComIF>               ComLibraryIFType;
        typedef ComIF                           ComIFType;

 
        Sensor3D(ComIF &comIF, SupportedSensorTypes_te sensorType, SupportedComLibraryInterfaceTypes_te comLibIFType = I2C_e) : bsc(), comLIF(comIF), sensor(sensorType, comLibIFType) {
        }


        void init() {
            bsc.init();
            sensor.init();

            initComLibIF(sensor.getSensorStruct(), comLIF);

            // if( sensor.getSensorStruct()->comIFType == I2C_e ) {
            //     bool b = initComLibIF(sensor.getSensorStruct(), comLIF);
            //     assert(b);
            // }
            // else {
            //     assert(0);
            // }

            // comLIF.init();

            sensor.setDefaultConfig();
        }


        void begin() {
            init();
        }


        void deinit() {
            sensor.deinit();
            comLIF.deinit();
            bsc.deinit();
        }


        void end() {
            deinit();
        }


        bool getTemperature(float *temperature) {
            return sensor.getTemperature(temperature);
        }


        bool getFieldValues(float *x, float *y, float *z) {
            return sensor.getFieldValues(x, y, z);
        }


        bool enableTemperature() {
            return sensor.enableTemperature();
        }


        bool disableTemperature() {
            return sensor.disableTemperature();
        }


        bool enableInterrupt() {
            return sensor.enableInterrupt();
        }


        bool disableInterrupt() {
            return sensor.disableInterrupt();
        }


        bool setPowerMode(uint8_t mode) {
            return sensor.setPowerMode(mode);
        }


        bool setIICAddress(uint8_t addr) {
            return sensor.setIICAddress(addr);
        }

        
        bool enableAngularMeasurement() {
            return sensor.enableAngularMeasurement();
        }


        bool disableAngularMeasurement() {
            return sensor.disableAngularMeasurement();
        }


        bool setTriggerBits(uint8_t bits) {
            return sensor.setTriggerBits();
        }

        
        bool setUpdateRate(uint8_t bit) {
            return sensor.setUpdateRate(bit);
        }


        bool isWakeUpActive() {
            return sensor.isWakeUpActive();
        }


        bool enableWakeUpMode() {
            return sensor.enableWakeUpMode();
        }


        bool disableWakeUpMode() {
            return sensor.disableWakeUpMode();
        }

        bool setLowerWakeUpThresholdX(int16_t threshold) {
            return sensor.setLowerWakeUpThresholdX(threshold);
        }

        bool setLowerWakeUpThresholdY(int16_t threshold) {
            return sensor.setLowerWakeUpThresholdY(threshold);
        }

        bool setLowerWakeUpThresholdZ(int16_t threshold) {
            return sensor.setLowerWakeUpThresholdZ(threshold);
        }

        bool setUpperWakeUpThresholdX(int16_t threshold) {
            return sensor.setUpperWakeUpThresholdX(threshold);
        }

        bool setUpperWakeUpThresholdY(int16_t threshold) {
            return sensor.setUpperWakeUpThresholdY(threshold);
        }

        bool setUpperWakeUpThresholdZ(int16_t threshold) {
            return sensor.setUpperWakeUpThresholdZ(threshold);
        }

        bool setWakeUpThresholds(int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th) {
            return sensor.setWakeUpThresholds(xl_th, xh_th, yl_th, yh_th, zl_th, zh_th);
        }


    private:

        BoardSupportClassType  bsc;
        ComLibraryIFType       comLIF;
        SensorClass            sensor;
};

#endif // MAGNETIC_3D_HPP
