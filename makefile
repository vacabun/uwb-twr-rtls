clean:
	rm -rf $(CURDIR)/build
tag:
	west build --pristine always -o=-j64 -- -DTAG_DEF=ON -DANCHOR_DEF=OFF 
anchor:
	west build -- -DTAG_DEF=OFF -DANCHOR_DEF=ON