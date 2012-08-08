all: installed

CUR_DIR = $(shell rospack find lcm)
GIT_DIR = build/april
GIT_URL = git://april.eecs.umich.edu/home/git/april.git
GIT_REVISION = eafdf0739d0adaf1e61cb2a38fa973b7c75d7577
GIT_PATCH = openjdk-6-patch.diff
include $(shell rospack find mk)/git_checkout.mk

installed: $(GIT_DIR) patched
	cd $(GIT_DIR)/java && CLASSPATH=${CLASSPATH}:/usr/share/java/gluegen-rt.jar:`rospack find lcm`/share/java/lcm.jar PATH=${PATH}:`rospack find lcm`/bin ant
	mvn install:install-file -Dfile=./build/april/java/april.jar -DgroupId=april -DartifactId=april -Dversion=0.0.0-SNAPSHOT -Dpackaging=jar
	touch installed

clean:
	cd $(GIT_DIR)/java && ant clean
	rm -rf installed patched

wipe: clean
	rm -rf $(GIT_DIR) build rospack_nosubdirs
