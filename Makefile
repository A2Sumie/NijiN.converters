.PHONY: all mcu sil check test clean

all mcu sil check test clean:
	$(MAKE) -C se-nz $@
