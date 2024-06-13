FQBN  ?=
PORT  ?=
TESTS ?=

$(info FQBN : $(FQBN))
$(info PORT : $(PORT))


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


test_all: TESTS=$(TESTS_NEEDS_SENSOR) $(TESTS_NO_SENSOR)
test_needsSensor: TESTS=$(TESTS_NEEDS_SENSOR) $(TEST_COMMON_NEEDS_SENSOR)
test: TESTS=$(TESTS_NO_SENSOR)

test_all \
test_needsSensor \
test: unity flash


EXAMPLES = iic_c_style iic iic_with_wakeup 3iic 3iic_equal iic_ext_addr spi

# $(EXAMPLES): arduino compile


### Arduino targets
clean:
	-rm -rf build/* cppcheck_reports build.ino.*elf.* ./-lm.res log.[0-9]*
	find . -name '*ctu-info' -exec \rm {} \;


arduino: clean
	cp -r config/arduinoLibraryTemplate/* build
	find src -name '*.[hc]*' -a \! -path '*mtb*' -a \! -name 'main*' -print -exec cp {} build \;
	rm -f build/n_* build/*.save


iic_ext_addr: arduino
	cp examples/framework/arduino/read_iic_a1b6_extended_addresses.ino build/build.ino


iic_c_style: arduino
	cp examples/framework/arduino/read_iic_sensor_c_style.ino build/build.ino
 

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


unity: arduino
	find ../../unity/Unity-master -name '*.[hc]' \( -path '*extras*' -a -path '*src*' -or -path '*src*' -a \! -path '*example*' \) -exec \cp {} build \;
	find test/unit/src -name '*.[hc]*' -a \! -path '*mtb*' -exec \cp {} build \;
	cp test/unit/src/framework/arduino/Test_main.ino build/build.ino



compile:
ifeq ($(FQBN),)
	$(error "Must set variable FQBN in order to be able to compile Arduino sketches !")
else
	arduino-cli.exe compile --clean --log --warnings all --fqbn $(FQBN) \
	                        --build-property compiler.c.extra_flags="\"-DUNITY_INCLUDE_CONFIG_H=1\"" \
							--build-property compiler.cpp.extra_flags="$(TESTS)" \
			        build
endif


compileLTO:
ifeq ($(FQBN),)
	$(error "Must set variable FQBN in order to be able to compile Arduino sketches !")
else
# switch to -std=c23 whenever XMCLib is conforming; currently neither c99 nor c11 work !
# CAUTION : only use '=' when assigning values to vars, not '+='
	arduino-cli.exe compile --clean --log --warnings all --fqbn $(FQBN) \
							--build-property "compiler.c.extra_flags=\"-DUNITY_INCLUDE_CONFIG_H=1\" -DNDEBUG -flto -fno-fat-lto-objects -Wextra -Wall -Wfloat-equal -Wconversion -Wredundant-decls -Wswitch-default -Wdouble-promotion -Wpedantic -Wunreachable-code -fanalyzer -std=c20" \
							--build-property compiler.cpp.extra_flags="$(TESTS) -DNDEBUG -flto -fno-fat-lto-objects -Wextra -Wall -Wfloat-equal -Wconversion -Wredundant-decls -Wswitch-default -Wdouble-promotion -Wpedantic -Wunreachable-code -std=c++20 -fanalyzer " \
							--build-property compiler.ar.cmd=arm-none-eabi-gcc-ar \
							--build-property compiler.libraries.ldflags=-lstdc++ \
							--build-property compiler.arm.cmsis.path="-isystem{compiler.xmclib_include.path}/XMCLib/inc -isystem{compiler.dsp_include.path} -isystem{compiler.nn_include.path} -isystem{compiler.cmsis_include.path} -isystem{compiler.xmclib_include.path}/LIBS -isystem{build.variant.path} -isystem{build.variant.config_path}" \
							--build-property compiler.usb.path="-isystem{runtime.platform.path}/cores/usblib -isystem{runtime.platform.path}/cores/usblib/Common -isystem{runtime.platform.path}/cores/usblib/Class -isystem{runtime.platform.path}/cores/usblib/Class/Common -isystem{runtime.platform.path}/cores/usblib/Class/Device -isystem{runtime.platform.path}/cores/usblib/Core -isystem{runtime.platform.path}/cores/usblib/Core/XMC4000" \
					build

#                           --build-property compiler.path="/mnt/c/Users/bargfred/arm-gnu-toolchain-13.2.Rel1-mingw-w64-i686-arm-none-eabi/bin/"
#                           --build-property compiler.path="/home/jensb/gcc/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin/"
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
flashLTO: compileLTO upload


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


# (make compile_examples) 2>&1 | tee log
# (make FQBN=Infineon:xmc:XMC1100_XMC2GO PORT=COM21 iic compile) 2>&1 | tee log
# (make FQBN=Infineon:xmc:XMC1100_XMC2GO PORT=COM21 iic compile) > log 2>&1

# ./filter.pl

compile_examples:
	make FQBN=Infineon:xmc:XMC1100_XMC2GO PORT=COM17 iic_c_style compile
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
	export PATH=~/cppcheck/cppcheck.danmar/:$PATH
	export RULE_TEXTS=/home/jensb/pull_requests/3d_magnetic/magnetic-3d-sensor-tlx493d/misra.txt					 
	~/cppcheck/cppcheck.danmar/cppcheck -i build -i config -i doc -i examples -i results -i reports_hml -i Unity \
                             -I./src/tlx493d -I./src/interfaces/c \
                             --checkers-report=cppcheck.checkers --check-level=exhaustive --xml --enable=all --inconclusive \
                             --addon=misra_local.py --addon=misc \
                             --max-configs=100 ./ 2> ./err.xml
	~/cppcheck/cppcheck.danmar/htmlreport/cppcheck-htmlreport --file=err.xml --title=TLx493D --report-dir=cppcheck_reports --source-dir=.
	firefox cppcheck_reports/index.html

