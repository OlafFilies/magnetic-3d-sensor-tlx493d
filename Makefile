FQBN ?=
PORT ?=

$(info FQBN : $(FQBN))
$(info PORT : $(PORT))


arduino_test_all: TESTS=-DTEST_TLE493D_A2B6 -DTEST_TLE493D_A2B6_NEEDS_SENSOR
arduino_test_needsSensor: TESTS=-DTEST_TLE493D_A2B6_NEEDS_SENSOR
arduino_test: TESTS=-DTEST_TLE493D_A2B6

arduino_test_all arduino_test_needsSensor arduino_test: arduino_unity arduino_flash


### Arduino targets
arduino_clean:
	-rm -rf build/*

arduino: arduino_clean
	cp -r config/arduinoLibraryTemplate/* build
	find src -name '*.[hc]*' -a \( \! -path '*mtb*' \) -a \( \! -name 'main*' \) -print -exec cp {} build \;


arduino_plain_c: arduino
	cp examples/arduino/simple_plain_c.ino build/build.ino
 

arduino_cpp: arduino
	cp examples/arduino/simple.ino build/build.ino


# example call : make FQBN=Infineon:xmc:XMC1100_XMC2GO PORT=COM16 TEST=TLE493D_A2B6 arduino_unity arduino_flash arduino_monitor
arduino_unity: arduino
	cp -r test/Unity/*.[hc] build
	cp test/src//Test_*.h build
	cp test/src/sensors/Test_*.h build
	cp test/src/arduino/Test_*.c* build
	cp test/src/arduino/Test_*.h* build
	cp test/src/arduino/Test_main.ino build/build.ino


# For WSL and Windows :
# download arduino-cli.exe from : https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip
arduino_prepare:
	arduino-cli.exe core update-index
	arduino-cli.exe core install Infineon:xmc
	arduino-cli.exe core update-index
	arduino-cli.exe core search Infineon
	arduino-cli.exe core list
	arduino-cli.exe board listall
	arduino-cli.exe board listall Infineon


arduino_compile:
ifeq ($(FQBN),)
	$(error "Must set variable FQBN in order to be able to compile Arduino sketches !")
else
# CAUTION : only use '=' when assigning values to vars, not '+='
	arduino-cli.exe compile --clean --log --warnings all --fqbn $(FQBN) --build-property "compiler.c.extra_flags=\"-DUNITY_INCLUDE_CONFIG_H=1\"" \
                                    		             --build-property compiler.cpp.extra_flags="$(TESTS)" build
#	                                        		      --build-property "compiler.cpp.extra_flags=\"$(TESTS)\"" build
#	                                             		 --build-property "compiler.cpp.extra_flags=\"-DTEST_TLE493D_A2B6=1\"" build
endif


arduino_upload:	
ifeq ($(PORT),)
	$(error "Must set variable PORT (Windows port naming convention, ie COM16) in order to be able to flash Arduino sketches !")
endif
ifeq ($(FQBN),)
	$(error "Must set variable FQBN in order to be able to flash Arduino sketches !")
else
	arduino-cli.exe upload -p $(PORT) --fqbn $(FQBN) build
endif


arduino_flash: arduino_compile arduino_upload


arduino_monitor:
ifeq ($(PORT),)
	$(error "Must set variable PORT (Windows port naming convention, ie COM16) in order to be able to flash Arduino sketches !")
endif
ifeq ($(FQBN),)
	$(error "Must set variable FQBN in order to be able to flash Arduino sketches !")
else
	arduino-cli.exe monitor -c baudrate=115200 -p $(PORT) --fqbn $(FQBN)
endif


# TODO: improve !
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
	cp examples/mtb/xmc/main_i2c.c ~/mtb3DMagneticSensors/src
	cp -r ~/mtb3DMagneticSensors/* /mnt/c/Users/$(WIN_USER)/mtw/$(BOARD)_I2C_Master_and_Slave/IFX3DMagneticSensors
endif


mtb_xmc_test: mtb_base
ifeq ($(WIN_USER),)
	$(error "Must set variable WIN_USER (Windows user account name) in order to be able to copy files to MTB directories !")
endif
ifeq ($(BOARD),)
	$(error "Must set variable BOARD (board name prefix, eg XMC4700_Relax or XMC1100_Boot) in order to be able to copy files to MTB directories !")
else
	cp test/src/mtb/xmc/ut_TLE493D_A2B6.c ~/mtb3DMagneticSensors/src
	cp -r test/Unity ~/mtb3DMagneticSensors/src
	cp -r ~/mtb3DMagneticSensors/* /mnt/c/Users/$(WIN_USER)/mtw/$(BOARD)_I2C_Master_and_Slave/IFX3DMagneticSensors
endif