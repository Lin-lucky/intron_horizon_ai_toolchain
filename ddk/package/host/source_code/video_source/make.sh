#!/bin/bash
usage() {
  echo "usage: bash make.sh [clean|plugin|noplugin] [x3] [debug|release|coverage]"
  exit 1
}

# set -x
SCRIPTS_DIR=$(cd `dirname $0`; pwd)
ALL_PROJECT_DIR=$PWD
ARCH="x3"
MODE="debug"
BIT="64bit"
PLUGIN_MODE=0

function build_clean() {
  cd ${ALL_PROJECT_DIR}
  rm build/ build_noplugin/ output/ deploy/ deploy_noplugin/ deploy_plugin/ -rf
}

if [ $# -ge 1 ];then
  HOBOT_COMPILE_MODE=${1}
  if [ x"${HOBOT_COMPILE_MODE}" == x"clean" ];then
    build_clean
    exit
  elif [ x"${HOBOT_COMPILE_MODE}" == x"plugin" ];then
    PLUGIN_MODE=1
  elif [ x"${HOBOT_COMPILE_MODE}" == x"noplugin" ];then
    PLUGIN_MODE=0
  else
    echo "error!!! compile cmd: ${HOBOT_COMPILE_MODE} is not supported"
  fi
fi

echo "PLUGIN_MODE: ${PLUGIN_MODE}"
#check params
# 1.check compile arch
if [ $# -ge 2 ];then
  ARCH=${2}
  if [ x"${ARCH}" != x"x3" ];then
    echo "error!!! compile architecture:${ARCH} is not supported"
    usage
  fi
else
  echo "use default ARCH:${ARCH}"
fi

# 2.check compile mode
if [ $# -ge 3 ]; then
  MODE=${3}
  if [ ${MODE} == "release" -o ${MODE} == "debug" -o ${MODE} == "coverage" ];then
    echo ""
  else
    echo "error!!! compile mode:${MODE} is not supported."
    usage
  fi
else
  echo "use default MODE:${MODE}"
fi

# 3.check compile bit
if [ $# -ge 4 ]; then
  BIT=${4}
  if [ ${BIT} == "32bit" -o ${BIT} == "64bit" ];then
    echo ""
  else
    echo "error!!! compile bit:${BIT} is not supported."
    usage
  fi
else
  echo "use default BIT:${BIT}"
fi

function cmake_build() {
  rm -rf build
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

function cmake_build_noplugin() {
  rm -rf build_noplugin
  mkdir build_noplugin
  cd build_noplugin
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
    if [ ${ARCH} == "x3" ]; then
      if [ ${PLUGIN_MODE} == 0 ]; then
        cmake_build_noplugin -DRELEASE_LIB=OFF -DPLATFORM_X3=ON -DPLUGIN_MODE=OFF
      else
        cmake_build -DRELEASE_LIB=OFF -DPLATFORM_X3=ON -DPLUGIN_MODE=ON
      fi
    fi
  elif [ ${MODE} == "coverage" ]; then
    if [ ${ARCH} == "x3" ]; then
      if [ ${PLUGIN_MODE} == 0 ]; then
        cmake_build_noplugin -DRELEASE_LIB=OFF -DPLATFORM_X3=ON -DPLUGIN_MODE=OFF -DCOVERAGE=ON
      else
        cmake_build -DRELEASE_LIB=OFF -DPLATFORM_X3=ON -DPLUGIN_MODE=ON -DCOVERAGE=ON
      fi
    fi
  else # mode == release
    if [ ${ARCH} == "x3" ]; then
      if [ ${PLUGIN_MODE} == 0 ]; then
        cmake_build_noplugin -DRELEASE_LIB=ON -DPLATFORM_X3=ON -DPLUGIN_MODE=OFF
      else
        cmake_build -DRELEASE_LIB=ON -DPLATFORM_X3=ON -DPLUGIN_MODE=ON
      fi
    fi
  fi
}


function prepare_deploy() {
  if [ ${PLUGIN_MODE} == 0 ];then
    mkdir -p deploy_noplugin
    cp ./output/* deploy_noplugin/ -R
    cp ./run_coverage_noplugin.sh ./deploy_noplugin
    cp ./sample/video_source/run.sh ./deploy_noplugin
    cp ./test/video_source/run_gtest.sh ./deploy_noplugin
  elif [ ${PLUGIN_MODE} == 1 ];then
    mkdir -p deploy_plugin
    cp ./output/* deploy_plugin/ -R
    cp ./run_coverage_plugin.sh ./deploy_plugin
    cp ./sample/video_source_plugin/run.sh ./deploy_plugin
    host_package_dir=${HOME}/.horizon/ddk/xj3_aarch64
    cp ${host_package_dir}/xproto/lib/libxproto.so ${ALL_PROJECT_DIR}/deploy_plugin/lib
    cp ${host_package_dir}/xstream/lib/libxstream.so ${ALL_PROJECT_DIR}/deploy_plugin/lib
  fi
  rm output/bin/ -rf
}

go_build_all
prepare_deploy
