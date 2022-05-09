#!/bin/bash
rm -rf output
rm -rf build
rm -rf image_utils*.tar.gz

mkdir -p build
cd build

# release .so
cmake ..
make -j
make install
cd ..

cd output
cur_time=$(date "+%Y%m%d%H%M%S")
tar -zcf image_utils${cur_time}.tar.gz image_utils
mv image_utils${cur_time}.tar.gz ../
cd ../
