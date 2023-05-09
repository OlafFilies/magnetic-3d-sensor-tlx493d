PATH_THIS_MAKEFILE    := $(abspath $(lastword $(MAKEFILE_LIST)))
DIR_OF_THIS_MAKEFILE  := $(dir $(MPY_PATH_TO_MTB_ADAPTER_MAKEFILE))


SOURCES = sensors/sensors.c \
		  sensors/sensors_common.c \
          sensors/sensors_gen_1/sensors_gen_1_common.c \
          sensors/sensors_gen_2/sensors_gen_2_common.c \
          sensors/sensors_gen_3/sensors_gen_3_common.c \
		  sensors/sensors_gen_1/TLE493D_A1B6/TLE493D_A1B6.c \
		  sensors/sensors_gen_1/TLV493D_A1B6/TLV493D_A1B6.c \
		  sensors/sensors_gen_2/TLE493D_P2B6/TLE493D_P2B6.c \
		  sensors/sensors_gen_2/TLE493D_W2B6/TLE493D_W2B6.c \
		  main.c



#CFLAGS = -Werror -std=c11 -Iinterface -Isensors -Isensors/sensors_gen_1 -Isensors/sensors_gen_2 -Isensors/sensors_gen_3
CFLAGS = -Werror -Wpedantic -std=c11 -I. -Iinterfaces -Isensors -Isensors/sensors_gen_1 -Isensors/sensors_gen_2 -Isensors/sensors_gen_3


#ifdef (ARDUINO_FRAMEWORK)
  SOURCES += framework/arduino/arduino.c
  CFLAGS += -Iframework/arduino
#else
#endif


all:
	gcc $(CFLAGS) $(SOURCES) -o main



arduinoLib:
	-rm -rf ../arduino3DMagneticSensors
	cp -r ../config/arduinoLibraryTemplate ../arduino3DMagneticSensors
	find . -name '*.[hC]' -print -exec cp {} ~/pull_requests/3d_magnetic/arduino3DMagneticSensors/src \;
	rm -rf /mnt/c/Users/bargfred/Documents/Arduino/libraries/IFX3DMagneticSensors
	mkdir /mnt/c/Users/bargfred/Documents/Arduino/libraries/IFX3DMagneticSensors
	cp -r ~/pull_requests/3d_magnetic/arduino3DMagneticSensors/* /mnt/c/Users/bargfred/Documents/Arduino/libraries/IFX3DMagneticSensors

