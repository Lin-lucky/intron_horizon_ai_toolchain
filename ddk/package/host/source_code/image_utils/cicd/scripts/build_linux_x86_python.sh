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

cp -rf build.properties.x86 build.properties
cp -rf CMakeLists.txt.x86 CMakeLists.txt
cd build
cmake .. ${cmake_opt[*]} -DBUILD_PYTHON=YES -DPYTHON_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())")  -DPYTHON_LIBRARY=$(python -c "import distutils.sysconfig as sysconfig; print(sysconfig.get_config_var('LIBDIR'))")
make -j4
make copy