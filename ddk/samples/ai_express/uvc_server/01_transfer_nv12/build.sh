#!/bin/bash
usage() {
  echo "usage: bash build.sh"
  exit 1
}

# set -x
SCRIPTS_DIR=$(cd `dirname $0`; pwd)
ALL_PROJECT_DIR=$PWD

function build_clean() {
  cd ${ALL_PROJECT_DIR}
  rm deploy/ -rf
}

# 1. check clean
if [ $# -eq 1 ];then
  HOBOT_COMPILE_MODE=${1}
  if [ x"${HOBOT_COMPILE_MODE}" == x"clean" ];then
    build_clean
    exit
  fi
fi

function cmake_build() {
  rm -rf build output
  mkdir build
  cd build
  cmake .. $*
  echo "##############################################"
  make -j16
  if [ $? -ne 0 ] ; then
    echo "failed to build"
    exit 1
  fi
  cd ${ALL_PROJECT_DIR}
}

function prepare_deploy() {
  mkdir -p deploy/bin
  mkdir -p deploy/data
  mkdir -p deploy/lib
  cp build/01_transfer_nv12 ./deploy/bin -rf
  cp ./data/* ./deploy/data -rf
  cp ~/.horizon/ddk/xj3_aarch64/uvc_server/lib/* ./deploy/lib -rf
  cp run.sh ./deploy -rf
}

build_clean
cmake_build
prepare_deploy