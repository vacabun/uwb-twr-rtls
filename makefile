clean:
	rm -rf $(CURDIR)/build
dw1000:
	cp $(CURDIR)/boards/stm32f401_mini_dw1000.overlay $(CURDIR)/boards/stm32f401_mini.overlay
	if grep -q 'CONFIG_DW3000=y' $(CURDIR)/prj.conf; then \
		sed -i 's/CONFIG_DW3000=y/CONFIG_DW3000=n/g' prj.conf; \
	fi
	if grep -q 'CONFIG_DW1000=n' $(CURDIR)/prj.conf; then \
		sed -i 's/CONFIG_DW1000=n/CONFIG_DW1000=y/g' prj.conf; \
	fi
dw3000:
	cp $(CURDIR)/boards/stm32f401_mini_dw3000.overlay $(CURDIR)/boards/stm32f401_mini.overlay
	if grep -q 'CONFIG_DW3000=n' $(CURDIR)/prj.conf; then \
		sed -i 's/CONFIG_DW3000=n/CONFIG_DW3000=y/g' prj.conf; \
	fi
	if grep -q 'CONFIG_DW1000=y' $(CURDIR)/prj.conf; then \
		sed -i 's/CONFIG_DW1000=y/CONFIG_DW1000=n/g' prj.conf; \
	fi
tag:
	west build -- -DTAG_DEF=ON -DANCHOR_DEF=OFF -DNODE_DEF=OFF
anchor:
	west build -- -DTAG_DEF=OFF -DANCHOR_DEF=ON -DNODE_DEF=OFF
node:
	west build -- -DTAG_DEF=OFF -DANCHOR_DEF=OFF -DNODE_DEF=ON
test:
	west build -- -DTAG_DEF=ON -DANCHOR_DEF=OFF
	west flash -r openocd --config boards/stlink.cfg
	west build -- -DTAG_DEF=OFF -DANCHOR_DEF=ON
	west flash -r openocd --config boards/daplink.cfg