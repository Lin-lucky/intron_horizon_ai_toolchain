#!/bin/bash
usage() {
  echo "usage: bash deploy.sh [clean|plugin|noplugin] [x3] [debug|release|coverage]"
  exit 1
}

function copy_deps_lib() {
  if [ ${PLUGIN_MODE} == 1 ];then
    host_package_dir=${HOME}/.horizon/ddk/xj3_aarch64
    cp ${host_package_dir}/xproto/lib/libxproto.so ${ALL_PROJECT_DIR}/output/lib
    cp ${host_package_dir}/xstream/lib/libxstream.so ${ALL_PROJECT_DIR}/output/lib
  fi
}

source ./make.sh $1 $2 $3
copy_deps_lib
