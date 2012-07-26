all: installed

CUR_DIR = $(shell rospack find lcm)
GIT_DIR = build/april
GIT_URL = git://april.eecs.umich.edu/home/git/april.git
include $(shell rospack find mk)/git_checkout.mk

installed: $(GIT_DIR) patched
	cd $(GIT_DIR)/java && CLASSPATH=${CLASSPATH}:/usr/share/java/gluegen-rt.jar:`rospack find lcm`/share/java/lcm.jar ant
	mvn install:install-file -Dfile=./build/april/java/april.jar -DgroupId=april -DartifactId=april -Dversion=0.0.0-SNAPSHOT -Dpackaging=jar
	touch installed

clean:
	cd $(GIT_DIR)/java && ant clean
	rm -rf installed

wipe: clean
	rm -rf $(GIT_DIR) build rospack_nosubdirs
