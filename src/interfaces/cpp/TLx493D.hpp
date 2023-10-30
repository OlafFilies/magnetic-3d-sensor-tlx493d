
#ifndef TLx493D_HPP
#define TLx493D_HPP


// std includes
#include <stdint.h>

// project cpp includes
#include "arduino_defines.h"

// project c includes


extern "C" {
    #include "sensor_types.h"
    #include "sensors_common.h"
}


class TLx493D {
   public:
        
        TLx493D(SupportedSensorTypes_te sensorType, SupportedComLibraryInterfaceTypes_te comLibIF) : sensor() {
            ::init(&sensor, sensorType);
            sensor.comIFType = comLibIF;
        }


        ~TLx493D() {
        }


        bool init() {
            return true;
        }


        bool begin() {
            return this->init();
        }


        bool deinit() {
            ::deinit(&sensor);
            return true;
        }


        bool end() {
            return this->deinit();
        }


        bool setDefaultConfig() {
            return ::setDefaultConfig(&sensor);
        }


        bool getTemperature(float *temperature) {
            return ::getTemperature(&sensor, temperature);
        }


        bool getFieldValues(float *x, float *y, float *z) {
            return ::getFieldValues(&sensor, x, y, z);
        }


        bool enableTemperature() {
            return ::enableTemperature(&sensor);
        }


        bool disableTemperature() {
            return ::disableTemperature(&sensor);
        }


        bool enableInterrupt() {
            return ::enableInterrupt(&sensor);
        }


        bool disableInterrupt() {
            return ::disableInterrupt(&sensor);
        }


        bool setPowerMode(uint8_t mode) {
            return ::setPowerMode(&sensor, mode);
        }

        
        bool setIICAddress(uint8_t addr) {
            return ::setIICAddress(&sensor, addr);
        }


        bool enableAngularMeasurement() {
            return ::enableAngularMeasurement(&sensor);
        }


        bool disableAngularMeasurement() {
            return ::disableAngularMeasurement(&sensor);
        }


        bool setTriggerBits(uint8_t bits) {
            return ::setTriggerBits(&sensor, bits);
        }

        
        bool setUpdateRate(uint8_t bit) {
            return ::setUpdateRate(&sensor, bit);
        }


        bool isWakeUpActive() {
            return ::isWakeUpActive(&sensor);
        }


        bool enableWakeUpMode() {
            return ::enableWakeUpMode(&sensor);
        }


        bool disableWakeUpMode() {
            return ::disableWakeUpMode(&sensor);
        }
        
        bool setLowerWakeUpThresholdX(int16_t threshold) {
            return ::setLowerWakeUpThresholdX(&sensor, threshold);
        }

        bool setLowerWakeUpThresholdY(int16_t threshold) {
            return ::setLowerWakeUpThresholdY(&sensor, threshold);
        }

        bool setLowerWakeUpThresholdZ(int16_t threshold) {
            return ::setLowerWakeUpThresholdZ(&sensor, threshold);
        }

        bool setUpperWakeUpThresholdX(int16_t threshold) {
            return ::setUpperWakeUpThresholdX(&sensor, threshold);
        }

        bool setUpperWakeUpThresholdY(int16_t threshold) {
            return ::setUpperWakeUpThresholdY(&sensor, threshold);
        }

        bool setUpperWakeUpThresholdZ(int16_t threshold) {
            return ::setUpperWakeUpThresholdZ(&sensor, threshold);
        }

        bool setWakeUpThresholds(int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th) {
            return ::setWakeUpThresholds(&sensor, xl_th, xh_th, yl_th, yh_th, zl_th, zh_th);
        }


        Sensor_ts *getSensorStruct() {
            return &sensor;
        }


        SupportedSensorTypes_te getSensorType() {
            return sensor.sensorType;
        }


        SupportedComLibraryInterfaceTypes_te getComLibIFType() {
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

        Sensor_ts  sensor;
};


#endif // TLx493D_HPP
