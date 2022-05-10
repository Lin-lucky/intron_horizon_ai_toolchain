#!/bin/bash

SCRIPTS_DIR=$(cd `dirname $0`; pwd)
ALL_PROJECT_DIR=$PWD
SOLUTION_ZOO_DIR=${ALL_PROJECT_DIR}/../..

function build_clean() {
  cd ${SOLUTION_ZOO_DIR}
  rm -rf common/wrapper/*/output
  rm -rf common/wrapper/*/*/output
  rm -rf common/xstream_methods/*/output
  rm -rf common/xproto_plugins/*/output
  cd ${ALL_PROJECT_DIR}
  rm build/ output/ deploy/ -rf
}
if [ $# -eq 1 ];then
  HOBOT_COMPILE_MODE=${1}
  if [ x"${HOBOT_COMPILE_MODE}" == x"clean" ];then
    build_clean
    exit
  else
    echo "error!!! compile cmd: ${HOBOT_COMPILE_MODE} is not supported"
  fi
fi

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
    cp ${host_package_dir}/video_source/lib/* ./deploy/lib -rf
    cp ${host_package_dir}/bpu_predict/lib/libbpu_predict.so ./deploy/lib -rf
    cp ${host_package_dir}/xproto/lib/libxproto.so ./deploy/lib -rf
    cp ${host_package_dir}/image_utils/lib/libimage_utils.so ./deploy/lib -rf
    cp ${host_package_dir}/xstream/lib/libxstream.so ./deploy/lib -rf
    cp ${host_package_dir}/appsdk/appuser/lib/sensorlib/libos8a10.so ./deploy/lib -rf
    cp ${host_package_dir}/dnn/lib/lib*.so ./deploy/lib -rf
    cp ../../common/deps/opencv/lib/libopencv_world.so.3.4 ./deploy/lib -rf
    cp ../../common/deps/protobuf/lib/libprotobuf.so.10 ./deploy/lib -rf
    cp ../../common/deps/uWS/lib/libuWS.so ./deploy/lib -rf
    cp ../../common/deps/libjpeg-turbo/lib/libturbojpeg.so.0 ./deploy/lib -rf
}

function cp_configs(){
    mkdir -p deploy/configs

    cp ./configs/video_source/* ./deploy/configs/ -rf
    cp ./output/yolov3_mobilenetv2_example ./deploy -rf
    chmod +x ./deploy/yolov3_mobilenetv2_example/yolov3_mobilenetv2_example
    cp run_example.sh deploy/
    cp ./output/visual_plugin/config/visualplugin_body.json ./deploy/configs/ -rf
    cp ./output/web_display_plugin/config/web_display_attribute.json ./deploy/configs/websocketplugin_attribute.json

    chmod +x ./deploy/run_example.sh

    cp ../../tools/webservice ./deploy -rf
    cp ../../tools/webservice/start_nginx.sh ./deploy -rf
    chmod +x ./deploy/start_nginx.sh
}

go_build_all
prepare_depend_so
cp_configs
