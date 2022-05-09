#!/bin/bash
rm -rf build
mkdir build
cd build
# release .so
cmake .. -DARM_ANDROID=ON -DRELEASE_LIB=ON -DBUILD_SHARED_LIBS=ON -DBUILD_ALL_ARCH=ON
make -j
make install
#--
# debug .so
cmake .. -DARM_ANDROID=ON -DRELEASE_LIB=OFF -DBUILD_SHARED_LIBS=ON -DBUILD_ALL_ARCH=ON
make -j
make install

# release .a
cmake .. -DARM_ANDROID=ON -DRELEASE_LIB=ON -DBUILD_SHARED_LIBS=OFF -DBUILD_ALL_ARCH=ON

make -j
make install

# debug .a
cmake .. -DARM_ANDROID=ON -DRELEASE_LIB=OFF -DBUILD_SHARED_LIBS=OFF -DBUILD_ALL_ARCH=ON
make -j
make install