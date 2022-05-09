#!/bin/bash
usage() {
  echo "usage: bash make.sh [release | debug | coverage]"
  exit 1
}

# set -x
SCRIPTS_DIR=$(cd `dirname $0`; pwd)
ALL_PROJECT_DIR=$PWD
MODE="debug"

function build_clean() {
  cd ${ALL_PROJECT_DIR}
  rm build/ output/  deploy/ -rf
}

# 1. check clean
if [ $# -eq 1 ];then
  HOBOT_COMPILE_MODE=${1}
  if [ x"${HOBOT_COMPILE_MODE}" == x"clean" ];then
    build_clean
    exit
  fi
fi

# 2. check compile mode
if [ $# -ge 1 ]; then
  MODE=${1}
  if [ ${MODE} == "release" -o ${MODE} == "debug" -o ${MODE} == "coverage" ];then
    echo "compile mode is:${MODE}"
  else
    echo "error!!! compile mode:${MODE} is not supported."
    usage
  fi
else
  echo "use default MODE:${MODE}"
fi

function cmake_build() {
  mkdir build
  cd build
  cmake .. $*
  echo "##############################################"
  echo $1
  make -j16
  if [ $? -ne 0 ] ; then
    echo "failed to build"
    exit 1
  fi
  # make copy
  make install
  cd ${ALL_PROJECT_DIR}
}

function go_build_all() {
  # mode == debug
  if [ ${MODE} == "debug" ]; then
    cmake_build -DRELEASE_LIB=OFF
  elif [ ${MODE} == "coverage" ]; then
    cmake_build -DRELEASE_LIB=OFF -DCOVERAGE=ON
  else # mode == release
    cmake_build -DRELEASE_LIB=ON
  fi
}

function prepare_configs() {
  mkdir -p deploy/lib
  mkdir -p deploy/tensor_task
  mkdir -p deploy/roi_task
  cp ./build/test/tensor_task/tensor_task ./deploy/tensor_task -rf
  cp ./test/tensor_task/configs ./deploy/tensor_task -rf
  cp ./build/test/roi_task/roi_resizer_task ./deploy/roi_task -rf
  cp ./test/roi_task/configs ./deploy/roi_task -rf
  cp ./build/libmodel_inference.so ./deploy/lib -rf
  cp ~/.horizon/ddk/xj3_aarch64/dnn/lib/lib* ./deploy/lib -rf
  cp ~/.horizon/ddk/xj3_aarch64/image_utils/lib/libimage_utils.so ./deploy/lib -rf
  cp ~/.horizon/ddk/xj3_aarch64/xstream/lib/libxstream.so ./deploy/lib -rf
  cp ~/.horizon/ddk/xj3_aarch64/bpu_predict/lib/libbpu_predict.so ./deploy/lib -rf
  cp ./test/roi_task/lib/libvideo_source.so ./deploy/lib -rf

  cp run_coverage.sh ./deploy
}

build_clean
go_build_all
prepare_configs