all: installed

TARBALL     = build/pl-5.8.1.tar.gz
TARBALL_URL = http://www.swi-prolog.org/download/stable/src/pl-5.8.1.tar.gz
UNPACK_CMD  = tar xzf
SOURCE_DIR  = build/pl-5.8.1
TARBALL_PATCH =
MD5SUM_FILE =

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) &&	./configure --prefix=$(CURDIR) && make && make install
	touch installed

clean:
	rm -rf $(CURDIR)/bin $(CURDIR)/lib $(CURDIR)/include $(CURDIR)/share installed

wipe: clean 
	rm -rf $(SOURCE_DIR)
	rm -rf build
