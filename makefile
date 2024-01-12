clean:
	rm -rf $(CURDIR)/build
	rm prj.conf
	rm prj.conf.template
	rm boards/stm32f401_mini.overlay
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