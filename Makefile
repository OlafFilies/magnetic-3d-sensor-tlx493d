FQBN ?=
PORT ?=
TEST ?=

$(info FQBN : $(FQBN))
$(info PORT : $(PORT))


# TEST_COMMON=-DTEST_SENSORS_COMMON \
# 		      -DTEST_SENSORS_GEN_1_COMMON \
# 			  -DTEST_SENSORS_GEN_2_COMMON \
# 			  -DTEST_SENSORS_GEN_3_COMMON

# TEST_COMMON_NEEDS_SENSOR=-DTEST_SENSORS_COMMONFUNCTIONS_NEEDS_SENSOR 

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
#test: unity compile


EXAMPLES = iic_plain_c iic iic_with_wakeup 3iic 3iic_equal iic_ext_addr spi

# $(EXAMPLES): arduino compile


### Arduino targets
clean:
	-rm -rf build/* cppcheck_reports build.ino.*elf.* ./-lm.res log log.[0-9]*
	find . -name '*ctu-info' -exec \rm {} \;


arduino: clean
	cp -r config/arduinoLibraryTemplate/* build
	find src -name '*.[hc]*' -a \( \! -path '*mtb*' \) -a \( \! -name 'main*' \) -print -exec cp {} build \;


iic_ext_addr: arduino
	cp examples/framework/arduino/read_iic_a1b6_extended_addresses.ino build/build.ino


iic_plain_c: arduino
	cp examples/framework/arduino/read_iic_sensor_plain_c.ino build/build.ino
 

spi: arduino
	cp examples/framework/arduino/read_spi_sensor.ino build/build.ino
 

iic: arduino
	cp examples/framework/arduino/read_iic_sensor.ino build/build.ino

iic_with_wakeup: arduino
	cp examples/framework/arduino/read_iic_sensor_with_wakeup.ino build/build.ino


3iic: arduino
	cp examples/framework/arduino/read_3_different_iic_sensors.ino build/build.ino


3iic_equal: arduino
	cp examples/framework/arduino/read_3_equal_iic_sensors.ino build/build.ino


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


comp_gcc:
	gcc -c -Wextra -Wall -Wfloat-equal -Wconversion -Wredundant-decls -Wswitch-default -Wdouble-promotion -Wpedantic -Wunreachable-code -std=c++17 foo.cpp -o foo.o


comp_clang:
	clang -c -Wextra -Wall -Wfloat-equal -Wconversion -Wredundant-decls -Wswitch-default -Wdouble-promotion -Wpedantic -Wno-c++98-compat -Wunreachable-code -std=c++17 foo.cpp -o foo.o


# 	#gcc -c -Wextra -Wall -Wfloat-equal -Wconversion -Wredundant-decls -Wswitch-default -Wdouble-promotion -Wpedantic -Wunreachable-code build/TLx493D_P2B6.c
# #	arm-none-eabi-gcc -c -Wextra -Wall -Wfloat-equal -Wconversion -Wredundant-decls -Wswitch-default -Wdouble-promotion -Wpedantic build/TLx493D_P2B6.c -o build/TLx493D_P2B6.o
# #	arm-none-eabi-gcc -c -Wextra -Wall -Wfloat-equal -Wconversion -Wredundant-decls -Wswitch-default -Wdouble-promotion -Wpedantic -Wno-c++98-compat -Wunreachable-code -std=c++17 foo.cpp -o foo.o
# #	clang -c -Wextra -Wall -Wfloat-equal -Wconversion -Wredundant-decls -Wswitch-default -Wdouble-promotion -Wpedantic -Wno-c++98-compat -Wunreachable-code -std=c++17 foo.cpp -o foo.o
# #	clang -c -Wextra -Wall -Wfloat-equal -Wconversion -Wredundant-decls -Wswitch-default -Wdouble-promotion -Wpedantic -Wno-c++98-compat -Wunreachable-code -std=c11 build/TLx493D_P2B6.c

# #	arm-none-eabi-gcc -c -Wextra -Wall -Wfloat-equal -Wconversion -Wredundant-decls -Wswitch-default -Wdouble-promotion -Wpedantic foo.cpp -o foo.o
# #	gcc -c -Wextra -Wall -Wfloat-equal -Wconversion -Wredundant-decls -Wswitch-default -Wdouble-promotion -Wpedantic build/TLx493D_P2B6.c -o build/TLx493D_P2B6.o
# #	gcc -c -Wall -Wpedantic build/TLx493D_P2B6.c -o build/TLx493D_P2B6.o


compile_examples:
	make FQBN=Infineon:xmc:XMC1100_XMC2GO PORT=COM17 iic_plain_c compile
	make FQBN=Infineon:xmc:XMC1100_XMC2GO PORT=COM17 iic compile
	make FQBN=Infineon:xmc:XMC4700_Relax_Kit PORT=COM18 iic_with_wakeup compile
	make FQBN=Infineon:xmc:XMC4700_Relax_Kit PORT=COM21 3iic compile
	make FQBN=Infineon:xmc:XMC4700_Relax_Kit PORT=COM24 3iic_equal compile
	make FQBN=Infineon:xmc:XMC1100_XMC2GO PORT=COM iic_ext_addr compile
	make FQBN=Infineon:xmc:XMC1100_XMC2GO PORT=COM22 spi compile



run_clang_tidy: C_CPP_SOURCES = $(shell find src -name \*.[hc]\*)


run_clang_tidy:
	$(info $(C_CPP_SOURCES))
	clang-tidy -header-filter=.* --extra-arg="-Isrc/tlx493d" --extra-arg="-Isrc/framework/arduino" --extra-arg="-Isrc/interfaces/c" --extra-arg="-Isrc/interfaces/cpp" --extra-arg="-I/mnt/c/Users/bargfred/AppData/Local/Arduino15/packages/Infineon/hardware/xmc/2.2.0/cores" $(C_CPP_SOURCES)


run_cppcheck:
	~/cppcheck/cppcheck.danmar/cppcheck -i build -i config -i doc -i examples -i results -i reports_hml -i Unity \
                             -I./src/tlx493d -I./src/interfaces/c \
                             --checkers-report=cppcheck.checkers --check-level=exhaustive --xml --enable=all --inconclusive \
                             --addon=misra_local.py --addon=misc \
                             --max-configs=100 ./ 2> ./err.xml
	~/cppcheck/cppcheck.danmar/htmlreport/cppcheck-htmlreport --file=err.xml --title=TLx493D --report-dir=cppcheck_reports --source-dir=.
	firefox cppcheck_reports/index.html



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