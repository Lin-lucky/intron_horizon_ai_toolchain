#!/bin/bash
rm -rf build
mkdir -p build
cd build
# debug so
cmake .. -DRELEASE_LIB=OFF -DBUILD_SHARED_LIBS=ON -DBUILD_ALL_ARCH=ON
make -j
cd ../

rm -rf output
mkdir output
mkdir output/test
mkdir output/tutorials
cp build/libxstream.so  output/test/
cp build/test/* output/test/
mkdir output/test/test
cp -rf test/configs* output/test/test/

cp build/libxstream.so  output/tutorials/
cp -rf  build/tutorials/* output/tutorials/
cp  -rf tutorials/stage1_hello_world/config output/tutorials/stage1_hello_world/
cp -rf tutorials/stage2_multithread/config output/tutorials/stage2_multithread/
cp -rf tutorials/stage3_update_parameter/config output/tutorials/stage3_update_parameter/
cp -rf tutorials/stage4_multisource/config output/tutorials/stage4_multisource/
cp -rf tutorials/stage5_timeout_alarm/config output/tutorials/stage5_timeout_alarm/
cp -rf tutorials/stage6_profile/config output/tutorials/stage6_profile/
cp -rf tutorials/stage7_node_output/config output/tutorials/stage7_node_output/
cp -rf tutorials/stage8_sub_workflow/config output/tutorials/stage8_sub_workflow/
cp -rf tutorials/stage9_disable_method/config output/tutorials/stage9_disable_method/
