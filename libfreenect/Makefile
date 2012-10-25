all: libfreenect

TARBALL = build/OpenKinect-libfreenect-9ab2c20.tar.gz
TARBALL_URL = http://cs.utexas.edu/~piyushk/share/OpenKinect-libfreenect-v0.1.2-17-g9ab2c20.tar.gz
SOURCE_DIR = build/OpenKinect-libfreenect-9ab2c20
MD5SUM_FILE = OpenKinect-libfreenect-v0.1.2-17-g9ab2c20.tar.gz.md5sum
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

libfreenect: $(SOURCE_DIR)/unpacked
	mkdir -p $(SOURCE_DIR)/build
	cd $(SOURCE_DIR)/build && cmake ../ 
	cd $(SOURCE_DIR)/build && make
	mkdir -p include/freenect
	cp $(SOURCE_DIR)/include/* include/
	#cp $(SOURCE_DIR)/wrappers/cpp/libfreenect.hpp include/freenect/
	#cp $(SOURCE_DIR)/wrappers/c_sync/libfreenect_sync.* include/freenect/
	ln -sf $(SOURCE_DIR)/build/bin bin
	ln -sf $(SOURCE_DIR)/build/lib lib
	touch libfreenect

clean:
	-rm -rf src $(SOURCE_DIR) libfreenect include bin lib

wipe: clean
	-rm -rf build
