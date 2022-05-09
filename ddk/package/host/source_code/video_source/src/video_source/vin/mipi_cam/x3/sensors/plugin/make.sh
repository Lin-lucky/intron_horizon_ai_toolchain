#!/bin/bash
usage() {
  echo "usage: bash make.sh [x3] [debug|release]"
  exit 1
}

# set -x
SCRIPTS_DIR=$(cd `dirname $0`; pwd)
ALL_PROJECT_DIR=$PWD
ARCH="x3"
MODE="release"
BIT="64bit"

function build_clean() {
  cd ${ALL_PROJECT_DIR}
  rm build/ output/  -rf
}

if [ $# -eq 1 ];then
  HOBOT_COMPILE_MODE=${1}
  if [ x"${HOBOT_COMPILE_MODE}" == x"clean" ];then
    build_clean
  fi
  exit
fi

#check params
# 1.check compile arch
if [ $# -ge 1 ];then
  ARCH=${1}
  if [ x"${ARCH}" != x"x3" ];then
    echo "error!!! compile architecture:${ARCH} is not supported"
    usage
  fi
else
  echo "use default ARCH:${ARCH}"
fi

# 2.check compile mode
if [ $# -ge 2 ]; then
  MODE=${2}
  if [ ${MODE} == "release" -o ${MODE} == "debug" ];then
    echo ""
  else
    echo "error!!! compile mode:${MODE} is not supported."
    usage
  fi
else
  echo "use default MODE:${MODE}"
fi

# 3.check compile bit
if [ $# -ge 3 ]; then
  BIT=${2}
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
  # make copy
  make install
  cd ${ALL_PROJECT_DIR}
}

build_clean
cmake_build
