#ifndef MAGNETIC_3D_HPP
#define MAGNETIC_3D_HPP

// std includes
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

 
        Sensor3D(ComIF &comIF, SupportedSensorTypes_te sensorType, SupportedComLibraryInterfaceTypes_te comLibIF) : bsc(), comLIF(comIF), sensor(sensorType, comLibIF) {
        }


        void init() {
            bsc.init();
            sensor.init();
            comLIF.init(sensor.getSensorStruct());

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


        bool updateGetTemperature(float *temperature) {
            return sensor.updateGetTemperature(temperature);
        }


        bool updateGetFieldValues(float *x, float *y, float *z) {
            return sensor.updateGetFieldValues(x, y, z);
        }


    private:

        BoardSupportClassType  bsc;
        ComLibraryIFType       comLIF;
        SensorClass            sensor;
};


#endif // MAGNETIC_3D_HPP
