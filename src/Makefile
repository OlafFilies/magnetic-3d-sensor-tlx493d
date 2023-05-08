
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
  SOURCES += arduino/arduino.c
  CFLAGS += -Iarduino
#else
#endif


all:
	gcc $(CFLAGS) $(SOURCES) -o main



# makearduino:
# 	cp ...
#	zip ...