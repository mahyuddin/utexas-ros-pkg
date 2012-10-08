all: threadpool

TARBALL = build/threadpool-threadpool.tar.gz
TARBALL_URL = http://threadpool.cvs.sourceforge.net/viewvc/threadpool/threadpool/?view=tar
SOURCE_DIR = build/threadpool
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

threadpool: $(SOURCE_DIR)/unpacked
	mkdir -p include/boost
	rm -rf include/boost/
	cp $(SOURCE_DIR)/boost include/boost -r

clean:
	-rm -rf src threadpool build
