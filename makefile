clean:
	rm -rf $(CURDIR)/build
dw1000:
	rm boards/stm32f401_mini.overlay
	cp boards/stm32f401_mini_dw1000.overlay boards/stm32f401_mini.overlay
dw3000:
	rm boards/stm32f401_mini.overlay
	cp boards/stm32f401_mini_dw3000.overlay boards/stm32f401_mini.overlay
tag:
	west build -- -DTAG_DEF=ON -DANCHOR_DEF=OFF
	west flash -r openocd --config boards/stlink.cfg
anchor:
	west build -- -DTAG_DEF=OFF -DANCHOR_DEF=ON
	west flash -r openocd --config boards/daplink.cfg
all:
	west build -- -DTAG_DEF=ON -DANCHOR_DEF=OFF
	west flash -r openocd --config boards/stlink.cfg
	west build -- -DTAG_DEF=OFF -DANCHOR_DEF=ON
	west flash -r openocd --config boards/daplink.cfg