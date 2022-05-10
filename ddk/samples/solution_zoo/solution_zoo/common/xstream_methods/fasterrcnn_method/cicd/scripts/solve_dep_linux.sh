#! /usr/bin/sh

chmod +x gradlew
cd build
cmake .. $1
make copy
make copy_models
make copy_bpu_config
make copy_vio_config
rm ../lib -rf
mkdir ../lib
#cp lib/libbpu_predict.so ../lib/  -rf
#cp lib/libhbrt_bernoulli_aarch64.so ../lib
cp lib/*.so* ../lib
rm ../lib/libcnn_intf.so
rm ../lib/libfb.so
rm ../lib/libvio.so
rm ../lib/libcam.so
exit 0
