#!/bin/make

# Author: Anne C. van Rossum
# Date: Mar. 8, 2012

all:
	@mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=Release $(CMAKE_FLAGS) .. $(FLAGS)
	cd build && make
	
debug:
	@mkdir -p build_debug
	#None Debug Release RelWithDebInfo MinSizeRel
	cd build && cmake -DCMAKE_BUILD_TYPE=Debug $(CMAKE_FLAGS) .. $(FLAGS)
	cd build && make
	
gumstix:
	@mkdir -p build_gumstix
	cd build_gumstix && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=../../cmake/Toolchain.cmake $(CMAKE_FLAGS) .. $(FLAGS)
	cd build_gumstix && make

clean:
	rm -f build/CMakeCache.txt
	cd build && make clean
	rm -f build_gumstix/CMakeCache.txt
	cd build_gumstix && make clean
	
install:
	echo "Do not install!"
#	cd build && make install

uninstall:
	echo "Do not install, I said! I don't know how to uninstall..."
