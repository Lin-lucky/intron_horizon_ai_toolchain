#!/bin/bash
rm -rf build
mkdir -p build
cd build
# debug so
cmake .. -DRELEASE_LIB=OFF -DBUILD_SHARED_LIBS=ON -DBUILD_ALL_ARCH=ON
make -j
cd -

rm -rf output
mkdir output
mkdir output/test
mkdir output/tutorials
cp build/libxproto.so  output/test/
cp build/test/* output/test/

cp build/libxproto.so  output/tutorials/
cp -rf build/tutorials/* output/tutorials/
