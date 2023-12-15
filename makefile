clean:
	rm -rf $(CURDIR)/build
dw1000:
	cp boards/stm32f401_mini_dw1000.overlay boards/stm32f401_mini.overlay
dw3000:
	cp boards/stm32f401_mini_dw3000.overlay boards/stm32f401_mini.overlay
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