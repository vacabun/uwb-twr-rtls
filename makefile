clean:
	rm -rf $(CURDIR)/build
	rm -f prj.conf
	rm -f prj.conf.template
	rm -f boards/stm32f401_mini.overlay
dw1000:
	cp $(CURDIR)/boards/stm32f401_mini_dw1000.overlay $(CURDIR)/boards/stm32f401_mini.overlay
	cp $(CURDIR)/prj.conf.dw1000 $(CURDIR)/prj.conf.template
dw3000:
	cp $(CURDIR)/boards/stm32f401_mini_dw3000.overlay $(CURDIR)/boards/stm32f401_mini.overlay
	cp $(CURDIR)/prj.conf.dw3000 $(CURDIR)/prj.conf.template
tag:
	cp $(CURDIR)/prj.conf.template $(CURDIR)/prj.conf
	echo "CONFIG_DEVICE_TYPE_TAG=y" >> $(CURDIR)/prj.conf
	echo "CONFIG_DEVICE_ADDRESS=1" >> $(CURDIR)/prj.conf
	west build
anchor:
	cp $(CURDIR)/prj.conf.template $(CURDIR)/prj.conf
	echo "CONFIG_DEVICE_TYPE_ANCHOR=y" >> $(CURDIR)/prj.conf
	echo "CONFIG_DEVICE_ADDRESS=4097" >> $(CURDIR)/prj.conf
	west build
node:
	cp $(CURDIR)/prj.conf.template $(CURDIR)/prj.conf
	echo "CONFIG_DEVICE_TYPE_NODE=y" >> $(CURDIR)/prj.conf
	echo "CONFIG_DEVICE_ADDRESS=1" >> $(CURDIR)/prj.conf
	west build
ds_twr:
	if grep -q 'CONFIG_SS_TWR=y' $(CURDIR)/prj.conf.template; then \
		sed -i 's/CONFIG_SS_TWR=y/CONFIG_DS_TWR=y/g' $(CURDIR)/prj.conf.template; \
	fi
ss_twr:
	if grep -q 'CONFIG_DS_TWR=y' $(CURDIR)/prj.conf.template; then \
		sed -i 's/CONFIG_DS_TWR=y/CONFIG_SS_TWR=y/g' $(CURDIR)/prj.conf.template; \
	fi
test:
	cp $(CURDIR)/prj.conf.template $(CURDIR)/prj.conf
	echo "CONFIG_DEVICE_TYPE_ANCHOR=y" >> $(CURDIR)/prj.conf
	echo "CONFIG_DEVICE_ADDRESS=4097" >> $(CURDIR)/prj.conf
	
	west build
	west flash -r openocd --config boards/stlink.cfg

	cp $(CURDIR)/prj.conf.template $(CURDIR)/prj.conf
	echo "CONFIG_DEVICE_TYPE_TAG=y" >> $(CURDIR)/prj.conf
	echo "CONFIG_DEVICE_ADDRESS=1" >> $(CURDIR)/prj.conf

	west build
	west flash -r openocd --config boards/daplink.cfg

test_node:
	cp $(CURDIR)/prj.conf.template $(CURDIR)/prj.conf
	echo "CONFIG_DEVICE_TYPE_NODE=y" >> $(CURDIR)/prj.conf
	echo "CONFIG_DEVICE_ADDRESS=1" >> $(CURDIR)/prj.conf

	west build
	west flash -r openocd --config boards/stlink.cfg

	cp $(CURDIR)/prj.conf.template $(CURDIR)/prj.conf
	echo "CONFIG_DEVICE_TYPE_NODE=y" >> $(CURDIR)/prj.conf
	echo "CONFIG_DEVICE_ADDRESS=2" >> $(CURDIR)/prj.conf
	west build
	west flash -r openocd --config boards/daplink.cfg

flash_stlink:
	west flash -r openocd --config boards/stlink.cfg
flash_daplink:
	west flash -r openocd --config boards/daplink.cfg