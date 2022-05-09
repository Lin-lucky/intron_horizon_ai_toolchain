#!/bin/bash
rm -rf build
mkdir -p build
cd build

# release .so
cmake .. -DRELEASE_LIB=ON -DBUILD_SHARED_LIBS=ON -DBUILD_ALL_ARCH=ON
make -j
make install

# debug .so
cmake .. -DRELEASE_LIB=OFF -DBUILD_SHARED_LIBS=ON -DBUILD_ALL_ARCH=ON
make -j
make install

# release .a
cmake .. -DRELEASE_LIB=ON -DBUILD_SHARED_LIBS=OFF -DBUILD_ALL_ARCH=ON
make -j
make install

# debug .a
cmake .. -DRELEASE_LIB=OFF -DBUILD_SHARED_LIBS=OFF -DBUILD_ALL_ARCH=ON
make -j
make install
