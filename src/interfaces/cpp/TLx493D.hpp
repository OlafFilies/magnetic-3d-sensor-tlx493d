
#ifndef TLx493D_HPP
#define TLx493D_HPP


// std includes
#include <cstdint>

// project cpp includes
#include "arduino_defines.h"

// project c includes


extern "C" {
    #include "sensor_types.h"
    #include "sensors_common.h"
}


class TLx493D {
   public:
        
        TLx493D(SupportedSensorTypes_te sensorType, SupportedComLibraryInterfaceTypes_te comLibIF) : _sensor() {
            ::init(&_sensor, sensorType, comLibIF);
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
            ::deinit(&_sensor);
            return true;
        }


        bool end() {
            return this->deinit();
        }


        template<typename ComIF, template<typename> typename ComLibrary> bool setDefaultConfig(ComLibrary<ComIF> &comLib) {
            uint8_t bufLen     = 0;
            uint8_t regMapSize = _sensor.regMapSize;
            uint8_t transBuffer[regMapSize];

            ::get1ByteModeBuffer(&_sensor, transBuffer, &bufLen);
            bool b = comLib.transfer(transBuffer, bufLen, _sensor.regMap, regMapSize);

            if( !b ) {
                return false;
            }

            ::getTemperatureMeasurementsBuffer(&_sensor, _sensor.regMap, transBuffer, &bufLen);
            b = comLib.transfer(transBuffer, bufLen, _sensor.regMap, regMapSize);

            return b;
        }


        bool getTemperature(float *temperature) {
            return ::getTemperature(&_sensor, temperature);
        }


        bool getFieldValues(float *x, float *y, float *z) {
            return ::getFieldValues(&_sensor, x, y, z);
        }


        bool updateGetFieldValues(float *x, float *y, float *z) {
            return ::updateGetFieldValues(&_sensor, x, y, z);
        }


        Sensor_ts *getSensorStruct() {
            return &_sensor;
        }


        SupportedSensorTypes_te getSensorType() {
            return _sensor.sensorType;
        }


        SupportedComLibraryInterfaceTypes_te getComLibIFType() {
            return _sensor.commIFType;
        }


        uint8_t *getRegisterMap() {
            return _sensor.regMap;
        }


        uint8_t getRegisterMapSize() {
            return _sensor.regMapSize;
        }


        uint8_t getI2CAddress() {
            return _sensor.comLibIFParams.i2c_params.address;
        }


    private:

        Sensor_ts  _sensor;
};


#endif // TLx493D_HPP

