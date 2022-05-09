#!/usr/bin/sh
echo "start coping runtime"
set -x
SCRIPTS_DIR=$(cd `dirname $0`; pwd)
echo ${SCRIPTS_DIR}
ALL_PROJECT_DIR=$PWD
PROJECT_DIR=${SCRIPTS_DIR}/../../
[ -e ${ALL_PROJECT_DIR}/release ] || mkdir ${ALL_PROJECT_DIR}/release
RELEASE_DIR=${ALL_PROJECT_DIR}/release/xstream-framework
rm ${RELEASE_DIR} -rf
mkdir ${RELEASE_DIR}
mkdir ${RELEASE_DIR}/bin
cp ${ALL_PROJECT_DIR}/build/bin/xstream_unit_test ${RELEASE_DIR}/bin -rf
cp ${ALL_PROJECT_DIR}/build/bin/bbox_filter_example ${RELEASE_DIR}/bin -rf
#cp ${ALL_PROJECT_DIR}/build/bin/c_bbox_filter_example ${RELEASE_DIR}/bin -rf
cp ${ALL_PROJECT_DIR}/build/bin/profiler_test ${RELEASE_DIR}/bin -rf
cp ${ALL_PROJECT_DIR}/build/bin/disable_method_test ${RELEASE_DIR}/bin -rf
mkdir ${RELEASE_DIR}/lib
cp ${ALL_PROJECT_DIR}/build/lib/libbbox_filter*.so ${RELEASE_DIR}/lib/ -rf
cp ${PROJECT_DIR}/config ${RELEASE_DIR}/ -rf
mkdir ${RELEASE_DIR}/test
cp ${PROJECT_DIR}/test/configs ${RELEASE_DIR}/test -rf
cp ${ALL_PROJECT_DIR}/build/bin/libhobotlog.so ${RELEASE_DIR}/lib -rf
cp ${SCRIPTS_DIR}/test_linux.sh ${RELEASE_DIR}/ -rf
echo "end coping runtime to " ${RELEASE_DIR}
exit 0

