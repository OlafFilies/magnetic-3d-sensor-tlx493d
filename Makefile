FQBN ?=
PORT ?=

$(info FQBN : $(FQBN))
$(info PORT : $(PORT))


# TEST_COMMON=-DTEST_SENSORS_COMMON \
# 			-DTEST_SENSORS_GEN_1_COMMON \
# 			-DTEST_SENSORS_GEN_2_COMMON \
# 			-DTEST_SENSORS_GEN_3_COMMON

# TEST_COMMON_NEEDS_SENSOR=-DTEST_SENSORS_COMMON_NEEDS_SENSOR \
# 						 -DTEST_SENSORS_GEN_1_COMMON_NEEDS_SENSOR \
# 						 -DTEST_SENSORS_GEN_2_COMMON_NEEDS_SENSOR \
# 			 			 -DTEST_SENSORS_GEN_3_COMMON_NEEDS_SENSOR

TESTS_NEEDS_SENSOR=-DTEST_TLx493D_A1B6_NEEDS_SENSOR \
                   -DTEST_TLx493D_A2B6_NEEDS_SENSOR \
				   -DTEST_TLx493D_P2B6_NEEDS_SENSOR \
				   -DTEST_TLx493D_W2B6_NEEDS_SENSOR \
	               -DTEST_TLx493D_W2BW_NEEDS_SENSOR

TESTS_NO_SENSOR=-DTEST_TLx493D_A1B6 \
                -DTEST_TLx493D_A2B6 \
				-DTEST_TLx493D_P2B6 \
				-DTEST_TLx493D_W2B6 \
			 	-DTEST_TLx493D_A2BW
				

A1B6_needsSensor: TESTS=-DTEST_TLx493D_A1B6 -DTEST_TLx493D_A1B6_NEEDS_SENSOR
A1B6: TESTS=-DTEST_TLx493D_A1B6

A2B6_needsSensor: TESTS=-DTEST_TLx493D_A2B6 -DTEST_TLx493D_A2B6_NEEDS_SENSOR
A2B6: TESTS=-DTEST_TLx493D_A2B6

P2B6_needsSensor: TESTS=-DTEST_TLx493D_P2B6 -DTEST_TLx493D_P2B6_NEEDS_SENSOR
P2B6: TESTS=-DTEST_TLx493D_P2B6

W2B6_needsSensor: TESTS=-DTEST_TLx493D_W2B6 -DTEST_TLx493D_W2B6_NEEDS_SENSOR
W2B6: TESTS=-DTEST_TLx493D_W2B6

A2BW_needsSensor: TESTS=-DTEST_TLx493D_W2BW -DTEST_TLx493D_W2BW_NEEDS_SENSOR
A2BW: TESTS=-DTEST_TLx493D_W2BW

A1B6_needsSensor A1B6 \
A2B6_needsSensor A2B6 \
P2B6_needsSensor P2B6 \
W2B6_needsSensor W2B6 \
A2BW_needsSensor W2BW : unity flash


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


plain_c: arduino
	cp examples/framework/arduino/read_sensors_plain_c.ino build/build.ino
 

cpp: arduino
	cp examples/framework/arduino/read_sensors.ino build/build.ino


# example call : make FQBN=Infineon:xmc:XMC1100_XMC2GO PORT=COM16 TEST=TLE493D_A2B6 unity flash monitor
unity: arduino
	cp -r test/unit/Unity/*.[hc] build
	cp test/unit/src/Test_*.h build
	cp test/unit/src/tlx493d/Test_*.h build
	cp test/unit/src/framework/arduino/Test_*.[hc]* build
	cp test/unit/src/framework/arduino/Test_main.ino build/build.ino


# For WSL and Windows :
# download arduino-cli.exe from : https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip
prepare:
	arduino-cli.exe core update-index
	arduino-cli.exe core install Infineon:xmc
	arduino-cli.exe core update-index
	arduino-cli.exe core search Infineon
	arduino-cli.exe core list
	arduino-cli.exe board listall
	arduino-cli.exe board listall Infineon


compile:
ifeq ($(FQBN),)
	$(error "Must set variable FQBN in order to be able to compile Arduino sketches !")
else
# CAUTION : only use '=' when assigning values to vars, not '+='
	arduino-cli.exe compile --clean --log --warnings all --fqbn $(FQBN) --build-property "compiler.c.extra_flags=\"-DUNITY_INCLUDE_CONFIG_H=1\"" \
                                    		             --build-property compiler.cpp.extra_flags="$(TESTS)" build
endif


upload:	
ifeq ($(PORT),)
	$(error "Must set variable PORT (Windows port naming convention, ie COM16) in order to be able to flash Arduino sketches !")
endif
ifeq ($(FQBN),)
	$(error "Must set variable FQBN in order to be able to flash Arduino sketches !")
else
	arduino-cli.exe upload -p $(PORT) --fqbn $(FQBN) build
endif


flash: compile upload


monitor:
ifeq ($(PORT),)
	$(error "Must set variable PORT (Windows port naming convention, ie COM16) in order to be able to flash Arduino sketches !")
endif
ifeq ($(FQBN),)
	$(error "Must set variable FQBN in order to be able to flash Arduino sketches !")
else
	arduino-cli.exe monitor -c baudrate=115200 -p $(PORT) --fqbn $(FQBN)
endif


# TODO: rework as for Arduino !
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