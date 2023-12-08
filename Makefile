FQBN ?=
PORT ?=
TEST ?=

$(info FQBN : $(FQBN))
$(info PORT : $(PORT))

# #################################
# # WSL setup with windows compile
# ARDUINOCLI = arduino-cli.exe

# #################################
# # Linux LabPC setup
ARDUINOCLI = arduino-cli

# TEST_COMMON=-DTEST_SENSORS_COMMON \
# 			-DTEST_SENSORS_GEN_1_COMMON \
# 			-DTEST_SENSORS_GEN_2_COMMON \
# 			-DTEST_SENSORS_GEN_3_COMMON

# TEST_COMMON_NEEDS_SENSOR=-DTEST_SENSORS_COMMON_NEEDS_SENSOR \
# 						 -DTEST_SENSORS_GEN_1_COMMON_NEEDS_SENSOR \
# 						 -DTEST_SENSORS_GEN_2_COMMON_NEEDS_SENSOR \
# 						 -DTEST_SENSORS_GEN_3_COMMON_NEEDS_SENSOR

TESTS_NEEDS_SENSOR=-DTEST_TLx493D_A1B6_NEEDS_SENSOR \
				   -DTEST_TLx493D_A2B6_NEEDS_SENSOR \
				   -DTEST_TLx493D_P2B6_NEEDS_SENSOR \
				   -DTEST_TLx493D_W2B6_NEEDS_SENSOR \
				   -DTEST_TLx493D_W2BW_NEEDS_SENSOR \
				   -DTEST_TLx493D_P3B6_NEEDS_SENSOR \
				   -DTEST_TLx493D_P3I8_NEEDS_SENSOR


TESTS_NO_SENSOR=-DTEST_TLx493D_A1B6 \
				-DTEST_TLx493D_A2B6 \
				-DTEST_TLx493D_P2B6 \
				-DTEST_TLx493D_W2B6 \
				-DTEST_TLx493D_W2BW \
				-DTEST_TLx493D_P3B6 \
				-DTEST_TLx493D_P3I8



A1B6_needsSensor: TESTS=-DTEST_TLx493D_A1B6 -DTEST_TLx493D_A1B6_NEEDS_SENSOR
A1B6: TESTS=-DTEST_TLx493D_A1B6

A2B6_needsSensor: TESTS=-DTEST_TLx493D_A2B6 -DTEST_TLx493D_A2B6_NEEDS_SENSOR
A2B6: TESTS=-DTEST_TLx493D_A2B6

P2B6_needsSensor: TESTS=-DTEST_TLx493D_P2B6 -DTEST_TLx493D_P2B6_NEEDS_SENSOR
P2B6: TESTS=-DTEST_TLx493D_P2B6

W2B6_needsSensor: TESTS=-DTEST_TLx493D_W2B6 -DTEST_TLx493D_W2B6_NEEDS_SENSOR
W2B6: TESTS=-DTEST_TLx493D_W2B6

W2BW_needsSensor: TESTS=-DTEST_TLx493D_W2BW -DTEST_TLx493D_W2BW_NEEDS_SENSOR
W2BW: TESTS=-DTEST_TLx493D_W2BW

P3B6_needsSensor: TESTS=-DTEST_TLx493D_P3B6 -DTEST_TLx493D_P3B6_NEEDS_SENSOR
P3B6: TESTS=-DTEST_TLx493D_P3B6

P3I8_needsSensor: TESTS=-DTEST_TLx493D_P3I8 -DTEST_TLx493D_P3I8_NEEDS_SENSOR
P3I8: TESTS=-DTEST_TLx493D_P3I8

A1B6_needsSensor A1B6 \
A2B6_needsSensor A2B6 \
P2B6_needsSensor P2B6 \
W2B6_needsSensor W2B6 \
W2BW_needsSensor W2BW \
P3B6_needsSensor P3B6 \
P3I8_needsSensor P3I8: unity flash


# sensor_common_needsSensor: TESTS=$(TEST_COMMON_NEEDS_SENSOR)
# sensor_common: TESTS=$(TEST_COMMON)


test_all: TESTS=$(TESTS_NEEDS_SENSOR) $(TESTS_NO_SENSOR)
test_needsSensor: TESTS=$(TESTS_NEEDS_SENSOR) $(TEST_COMMON_NEEDS_SENSOR)
test: TESTS=$(TESTS_NO_SENSOR)

test_all \
test_needsSensor \
test: unity flash


### Arduino targets
clean:
	-rm -rf build/*

arduino: clean
	cp -r config/arduinoLibraryTemplate/* build
	find src -name '*.[hc]*' -a \( \! -path '*mtb*' \) -a \( \! -name 'main*' \) -print -exec cp {} build \;


iic_plain_c: arduino
	cp examples/framework/arduino/read_iic_sensor_plain_c.ino build/build.ino


spi: arduino
	cp examples/framework/arduino/read_spi_sensor.ino build/build.ino


iic: arduino
	cp examples/framework/arduino/read_iic_sensor.ino build/build.ino


2iic: arduino
	cp examples/framework/arduino/read_2_iic_sensors.ino build/build.ino


# example call : make FQBN=Infineon:xmc:XMC1100_XMC2GO PORT=COM16 TEST=TLE493D_A2B6 unity flash monitor
unity: arduino
	cp -r test/unit/Unity/*.[hc] build
	cp test/unit/src/Test_*.h build
	cp test/unit/src/tlx493d/Test_*.h build
	cp test/unit/src/framework/arduino/Test_*.[hc]* build
	cp test/unit/src/framework/arduino/unity_ifx.cpp build
	cp test/unit/src/framework/arduino/Test_main.ino build/build.ino


compile:
ifeq ($(FQBN),)
	$(error "Must set variable FQBN in order to be able to compile Arduino sketches !")
else
# CAUTION : only use '=' when assigning values to vars, not '+='
	$(ARDUINOCLI) compile --clean --log --warnings all --fqbn $(FQBN) --build-property "compiler.c.extra_flags=\"-DUNITY_INCLUDE_CONFIG_H=1\"" \
														 --build-property compiler.cpp.extra_flags="$(TESTS)" build
endif


upload:
ifeq ($(PORT),)
	$(error "Must set variable PORT (Windows port naming convention, ie COM16) in order to be able to flash Arduino sketches !")
endif
ifeq ($(FQBN),)
	$(error "Must set variable FQBN in order to be able to flash Arduino sketches !")
else
	$(ARDUINOCLI) upload -p $(PORT) --fqbn $(FQBN) build
endif


flash: compile upload


monitor:
ifeq ($(PORT),)
	$(error "Must set variable PORT (Windows port naming convention, ie COM16) in order to be able to flash Arduino sketches !")
endif
ifeq ($(FQBN),)
	$(error "Must set variable FQBN in order to be able to flash Arduino sketches !")
else
	$(ARDUINOCLI) monitor -c baudrate=115200 -p $(PORT) --fqbn $(FQBN)
endif



# For WSL and Windows :
# download $(ARDUINOCLI) from : https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip
prepare:
	$(ARDUINOCLI) core update-index
	$(ARDUINOCLI) core install Infineon:xmc
	$(ARDUINOCLI) core update-index
	$(ARDUINOCLI) core search Infineon
	$(ARDUINOCLI) core list
	$(ARDUINOCLI) board listall
	$(ARDUINOCLI) board listall Infineon



# TODO: rework similar to Arduino !
### MTB targets
# ifeq ($(WIN_USER),)

# 	$(error "Must set variable WIN_USER (Windows user account name) in order to be able to copy files to Arduino or MTB directories !")

# elsifeq ($(BOARD),)

# 	$(error "Must set variable BOARD (board name prefix, eg XMC4700_Relax or XMC1100_Boot) in order to be able to copy files to Arduino or MTB directories !")

# endif
mtb_base:
ifeq ($(WIN_USER),)
	$(error "Must set variable WIN_USER (Windows user account name) in order to be able to copy files to MTB directories !")
endif
ifeq ($(BOARD),)
	$(error "Must set variable BOARD (board name prefix, eg XMC4700_Relax or XMC1100_Boot) in order to be able to copy files to Arduino or MTB directories !")
else
	-rm -rf ~/mtb3DMagneticSensors
	mkdir -p ~/mtb3DMagneticSensors/src
	find src -name '*.[hc]' -a \( \! -path '*arduino*' \) -print -exec cp {} ~/mtb3DMagneticSensors/src \;
	rm -rf /mnt/c/Users/$(WIN_USER)/mtw/$(BOARD)_I2C_Master_and_Slave/IFX3DMagneticSensors
	mkdir /mnt/c/Users/$(WIN_USER)/mtw/$(BOARD)_I2C_Master_and_Slave/IFX3DMagneticSensors
endif


mtb_xmc: mtb_base
ifeq ($(WIN_USER),)
	$(error "Must set variable WIN_USER (Windows user account name) in order to be able to copy files to MTB directories !")
endif
ifeq ($(BOARD),)
	$(error "Must set variable BOARD (board name prefix, eg XMC4700_Relax or XMC1100_Boot) in order to be able to copy files to MTB directories !")
else
	cp examples/framework/mtb/xmc/main_i2c.c ~/mtb3DMagneticSensors/src
	cp -r ~/mtb3DMagneticSensors/* /mnt/c/Users/$(WIN_USER)/mtw/$(BOARD)_I2C_Master_and_Slave/IFX3DMagneticSensors
endif


mtb_xmc_test: mtb_base
ifeq ($(WIN_USER),)
	$(error "Must set variable WIN_USER (Windows user account name) in order to be able to copy files to MTB directories !")
endif
ifeq ($(BOARD),)
	$(error "Must set variable BOARD (board name prefix, eg XMC4700_Relax or XMC1100_Boot) in order to be able to copy files to MTB directories !")
else
	cp test/unit/src/framework/mtb/xmc/ut_TLE493D_A2B6.c ~/mtb3DMagneticSensors/src
	cp -r test/unit/Unity ~/mtb3DMagneticSensors/src
	cp -r ~/mtb3DMagneticSensors/* /mnt/c/Users/$(WIN_USER)/mtw/$(BOARD)_I2C_Master_and_Slave/IFX3DMagneticSensors
endif