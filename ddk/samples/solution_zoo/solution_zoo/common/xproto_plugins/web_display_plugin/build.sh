function go_build_all(){
  rm build -rf
  rm output -rf
  mkdir build
  cd build
  cmake .. $*
  echo "##############################################"
  echo $1
  make -j
  if [ $? -ne 0 ] ; then
    echo "failed to build"
    exit 1
   fi
  make install
  cd -
}

function copy_depend_so(){
    mkdir output/lib
    host_package_dir=~/.horizon/ddk/xj3_aarch64
    cp ${host_package_dir}/bpu_predict/lib/* output/lib
    cp ${host_package_dir}/xproto/lib/libxproto.so output/lib
    cp ${host_package_dir}/image_utils/lib/libimage_utils.so output/lib
    cp ${host_package_dir}/xstream/lib/libxstream.so output/lib
    cp ../../deps/opencv/lib/libopencv_world.so.3.4 output/lib
}

function cp_config(){
    mkdir output/config
    cp ./example/config ./output -rf
    cp ./example/data ./output -rf
    cp ../models/cnn_method/* ./output/config/models
}
go_build_all -DPARENT_BUILD=OFF
#copy_depend_so
#cp_config