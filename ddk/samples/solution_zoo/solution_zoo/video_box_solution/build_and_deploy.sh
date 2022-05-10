function go_build_all(){
  rm build -rf
  rm output -rf
  rm deploy -rf
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

function prepare_depend_so(){
    mkdir -p deploy/lib
    find ./output -name "*.so*" | xargs -i cp {} ./deploy/lib -rf
    
    host_package_dir=~/.horizon/ddk/xj3_aarch64

    cp ${host_package_dir}/model_inference/lib/* ./deploy/lib -rf
    cp ${host_package_dir}/bpu_predict/lib/libbpu_predict.so ./deploy/lib -rf
    cp ${host_package_dir}/xproto/lib/libxproto.so ./deploy/lib -rf
    cp ${host_package_dir}/image_utils/lib/libimage_utils.so ./deploy/lib -rf
    cp ${host_package_dir}/xstream/lib/libxstream.so ./deploy/lib -rf
    cp ${host_package_dir}/appsdk/appuser/lib/sensorlib/libos8a10.so ./deploy/lib -rf
    cp ${host_package_dir}/dnn/lib/lib*.so ./deploy/lib -rf
    cp ../common/deps/opencv/lib/libopencv_world.so.3.4 ./deploy/lib -rf
    cp ../common/deps/protobuf/lib/libprotobuf.so.10 ./deploy/lib -rf
    cp ../common/deps/uWS/lib/libuWS.so ./deploy/lib -rf
    cp ../common/deps/live555/lib/* ./deploy/lib -rf
    cp ../common/deps/libjpeg-turbo/lib/libturbojpeg.so.0 ./deploy/lib -rf
    cp ../common/deps/zeroMQ/lib/libzmq.so.5 ./deploy/lib -rf
    cp ../common/deps/xwarehouse/lib/libxwarehouse.so ./deploy/lib -rf
    cp ./output/video_box ./deploy -rf
}

function cp_configs(){
    #cp ./output/video_source_plugin/configs ./deploy -rf
    cp ./data/models ./deploy -rf
    cp ./data/test.264 ./deploy -rf
    cp run_video_box.sh ./deploy
    chmod +x ./deploy/run_video_box.sh
}

go_build_all -DPARENT_BUILD=ON -DRELEASE_LIB=ON
prepare_depend_so
cp_configs