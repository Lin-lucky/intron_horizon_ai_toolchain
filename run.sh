# define gcc path for arm
LINARO_GCC_ROOT=/opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/

DIR=$(cd "$(dirname "$0")";pwd)

export CC=${LINARO_GCC_ROOT}/bin/aarch64-linux-gnu-gcc
export CXX=${LINARO_GCC_ROOT}/bin/aarch64-linux-gnu-g++

rm -rf build_arm
mkdir build_arm
cd build_arm

cmake ${DIR}

make -j8

