all: installed

CUR_DIR = $(shell rospack find lcm)
SVN_DIR = build/lcm
SVN_URL = http://lcm.googlecode.com/svn/trunk
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched
	cd $(SVN_DIR) && ./bootstrap.sh
	cd $(SVN_DIR) && ./configure --prefix=$(CUR_DIR)
	cd $(SVN_DIR) && make install
	touch installed

clean:
	cd $(SVN_DIR) && make clean
	rm -rf bin lib share include installed patched

wipe: clean
	rm -rf $(SVN_DIR) build rospack_nosubdirs
