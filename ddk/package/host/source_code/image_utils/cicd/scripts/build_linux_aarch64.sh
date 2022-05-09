#! /usr/bin/sh

rm -rf build
mkdir build

index=1
for param in $*
do
  if [ $param == "bgr2i420_with_libyuv" ]
  then
    cmake_opt[index]=-DBGR2I420_WITH_LIBYUV=ON
  fi
  index=`expr $index + 1`
done
echo "cmake options: ${cmake_opt[*]}"

cp -rf build.properties.aarch64 build.properties
cp -rf CMakeLists.txt.aarch64 CMakeLists.txt
cd build
cmake .. ${cmake_opt[*]}
make -j4
make copy
