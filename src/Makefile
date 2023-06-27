
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


mtb_xmc:
	-rm -rf ~/mtb3DMagneticSensors
	mkdir -p ~/mtb3DMagneticSensors/src
	find . -name '*.[hc]' -a \( \! -path '*arduino*' \) -print -exec cp {} ~/mtb3DMagneticSensors/src \;
	cp ../examples/mtb/xmc/*_i2c.c ~/mtb3DMagneticSensors/src
	rm -rf /mnt/c/Users/$(WIN_USER)/mtw/XMC4700_Relax_I2C_Master_and_Slave/IFX3DMagneticSensors
	mkdir /mnt/c/Users/$(WIN_USER)/mtw/XMC4700_Relax_I2C_Master_and_Slave/IFX3DMagneticSensors
	cp -r ~/mtb3DMagneticSensors/* /mnt/c/Users/$(WIN_USER)/mtw/XMC4700_Relax_I2C_Master_and_Slave/IFX3DMagneticSensors


mtb_xmc_test:
	-rm -rf ~/mtb3DMagneticSensors
	mkdir -p ~/mtb3DMagneticSensors/src
	find . -name '*.[hc]' -a \( \! -path '*arduino*' \) -print -exec cp {} ~/mtb3DMagneticSensors/src \;
	cp ../examples/mtb/xmc/relax_*.c ~/mtb3DMagneticSensors/src
	cp ../test/src/ut_i2c.c ~/mtb3DMagneticSensors/src
	cp -r ../test/Unity ~/mtb3DMagneticSensors/src
	rm -rf /mnt/c/Users/$(WIN_USER)/mtw/XMC4700_Relax_I2C_Master_and_Slave/IFX3DMagneticSensors
	mkdir /mnt/c/Users/$(WIN_USER)/mtw/XMC4700_Relax_I2C_Master_and_Slave/IFX3DMagneticSensors
	cp -r ~/mtb3DMagneticSensors/* /mnt/c/Users/$(WIN_USER)/mtw/XMC4700_Relax_I2C_Master_and_Slave/IFX3DMagneticSensors
