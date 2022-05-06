all:
	$(MAKE) -C build -j $(shell nproc)

prepare:
	rm -rf build && \
	mkdir build && \
    cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/package/homfa -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10 ..

install:
	$(MAKE) -C build install

.PHONY: all prepare install
