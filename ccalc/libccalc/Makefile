all: installed

TARBALL     = build/ccalc-2.0r1.tar.gz
TARBALL_URL = http://www.cs.utexas.edu/~tag/cc/ccalc.2.0/ccalc-2.0r1.tar.gz
UNPACK_CMD  = tar xvzf
SOURCE_DIR  = build/ccalc
TARBALL_PATCH =
MD5SUM_FILE =

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	touch installed

clean:
	rm -rf $(CURDIR)/lib $(CURDIR)/include installed

wipe: clean
	rm -rf $(SOURCE_DIR)
	rm -rf build
