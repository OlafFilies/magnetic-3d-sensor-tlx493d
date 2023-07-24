
ifeq ($(WIN_USER),)
    $(error "Must set variable WIN_USER (Windows user account name) in order to be able to copy files to Arduino or MTB directories !")
endif


arduino:
	-rm -rf ~/arduino3DMagneticSensors
	cp -r ../config/arduinoLibraryTemplate ~/arduino3DMagneticSensors
	find . -name '*.[hc]*' -a \( \! -path '*mtb*' \) -a \( \! -name 'main*' \) -print -exec cp {} ~/arduino3DMagneticSensors/src \;
	rm -rf /mnt/c/Users/$(WIN_USER)/Documents/Arduino/libraries/IFX3DMagneticSensors
	mkdir /mnt/c/Users/$(WIN_USER)/Documents/Arduino/libraries/IFX3DMagneticSensors
	cp -r ~/arduino3DMagneticSensors/* /mnt/c/Users/$(WIN_USER)/Documents/Arduino/libraries/IFX3DMagneticSensors


arduino_plain_c: arduino
	cp ../examples/arduino/simple_plain_c.ino /mnt/c/Users/$(WIN_USER)/Documents/Arduino/3dmagnetic/3dmagnetic.ino


arduino_cpp: arduino
	cp ../examples/arduino/simple.ino /mnt/c/Users/$(WIN_USER)/Documents/Arduino/3dmagnetic/3dmagnetic.ino


arduino_unity:
	rm -rf ~/arduino3DMagneticSensors
	cp -r ../config/arduinoLibraryTemplate ~/arduino3DMagneticSensors
	find . -name '*.[hc]*' -a \( \! -path '*mtb*' \) -a \( \! -name 'main*' \) -print -exec cp {} ~/arduino3DMagneticSensors/src \;
	cp -r ../test/Unity/* ~/arduino3DMagneticSensors/src
	cp ../test/src/arduino/*.h ~/arduino3DMagneticSensors/src
	rm -rf /mnt/c/Users/$(WIN_USER)/Documents/Arduino/libraries/IFX3DMagneticSensors
	mkdir /mnt/c/Users/$(WIN_USER)/Documents/Arduino/libraries/IFX3DMagneticSensors
	rm -rf /mnt/c/Users/$(WIN_USER)/Documents/Arduino/ut_TLE493D_A2B6
	mkdir /mnt/c/Users/$(WIN_USER)/Documents/Arduino/ut_TLE493D_A2B6
	cp ../test/src/arduino/ut_TLE493D_A2B6.ino /mnt/c/Users/$(WIN_USER)/Documents/Arduino/ut_TLE493D_A2B6/ut_TLE493D_A2B6.ino
	cp -r ~/arduino3DMagneticSensors/* /mnt/c/Users/$(WIN_USER)/Documents/Arduino/libraries/IFX3DMagneticSensors


mtb_base:
	-rm -rf ~/mtb3DMagneticSensors
	mkdir -p ~/mtb3DMagneticSensors/src
	find . -name '*.[hc]' -a \( \! -path '*arduino*' \) -print -exec cp {} ~/mtb3DMagneticSensors/src \;
	rm -rf /mnt/c/Users/$(WIN_USER)/mtw/XMC4700_Relax_I2C_Master_and_Slave/IFX3DMagneticSensors
	mkdir /mnt/c/Users/$(WIN_USER)/mtw/XMC4700_Relax_I2C_Master_and_Slave/IFX3DMagneticSensors


mtb_xmc: mtb_base
	cp ../examples/mtb/xmc/main_i2c.c ~/mtb3DMagneticSensors/src
	cp -r ~/mtb3DMagneticSensors/* /mnt/c/Users/$(WIN_USER)/mtw/XMC4700_Relax_I2C_Master_and_Slave/IFX3DMagneticSensors


mtb_xmc_test: mtb_base
	cp ../test/src/mtb/xmc/ut_TLE493D_A2B6.c ~/mtb3DMagneticSensors/src
	cp -r ../test/Unity ~/mtb3DMagneticSensors/src
	cp -r ~/mtb3DMagneticSensors/* /mnt/c/Users/$(WIN_USER)/mtw/XMC4700_Relax_I2C_Master_and_Slave/IFX3DMagneticSensors
