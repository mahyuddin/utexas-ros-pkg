SOURCES=$(shell find src/main -name *.java)
PACKAGE=april_tags_node

all: installed

installed: $(SOURCES)
	`rosstack find rosjava_core`/gradlew installApp
	mkdir -p bin
	ln -sf ../build/install/$(PACKAGE)/bin/$(PACKAGE) bin/execute
	chmod a+x bin/execute
	touch installed rospack_nosubdirs

clean:
	`rosstack find rosjava_core`/gradlew clean
	rm -rf bin installed rospack_nosubdirs
