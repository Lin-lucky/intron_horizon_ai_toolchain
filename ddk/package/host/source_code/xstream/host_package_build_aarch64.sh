#!/bin/bash
rm -rf output
rm -rf build
rm -rf xstream*.tar.gz

mkdir -p build
cd build

# release .so
cmake .. -DRELEASE_LIB=ON -DBUILD_SHARED_LIBS=ON -DBUILD_ALL_ARCH=OFF -DINSTALL_TUTORIALS=OFF
make -j
make install
cd ..

cd output
cur_time=$(date "+%Y%m%d%H%M%S")
tar -zcf xstream${cur_time}.tar.gz xstream
mv xstream${cur_time}.tar.gz ../
cd ../
