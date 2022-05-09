#!/bin/bash
rm -rf build
mkdir -p build
cd build
cmake .. -DX86_ARCH=ON -DX86_CENTOS=ON -DBUILD_ALL_ARCH=ON
make -j
make install

# debug so
cmake .. -DX86_ARCH=ON -DX86_CENTOS=ON -DRELEASE_LIB=OFF -DBUILD_ALL_ARCH=ON
make -j
make install

# release .a
cmake .. -DX86_ARCH=ON -DX86_CENTOS=ON -DRELEASE_LIB=ON -DBUILD_SHARED_LIBS=OFF -DBUILD_ALL_ARCH=ON

make -j
make install

# debug .a
cmake .. -DX86_ARCH=ON -DX86_CENTOS=ON -DRELEASE_LIB=OFF -DBUILD_SHARED_LIBS=OFF -DBUILD_ALL_ARCH=ON
make -j
make install
