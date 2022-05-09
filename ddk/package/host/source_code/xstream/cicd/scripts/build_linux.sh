#! /usr/bin/bash -ex

rm -rf build
mkdir build

index=1

for param in $*
do
  if [ $param == "cpu" ]
  then
    cmake_opt[index]=-DGPU_VER=false
    index=`expr $index + 1`
    cmake_opt[index]=-DTENSORRT_VER=false
  elif [ $param == "gpu" ]
  then
    cmake_opt[index]=-DGPU_VER=true
    index=`expr $index + 1`
    cmake_opt[index]=-DTENSORRT_VER=false
  elif [ $param == "tensorrt" ]
  then
    cmake_opt[index]=-DGPU_VER=true
    index=`expr $index + 1`
    cmake_opt[index]=-DTENSORRT_VER=true
  elif [ $param == "coverage" ]
  then
    cmake_opt[index]=-DCOVERAGE_TEST=true
  elif [ $param == "secure" ]
  then
    cmake_opt[index]=-DSECURE_VER=true
  elif [ $param == "debug" ]
  then
    cmake_opt[index]=-DRELEASE_LIB=off
  elif [ $param == "verbose" ]
  then
    cmake_opt[index]=-DVERBOSE_LOG=true
  fi
  index=`expr $index + 1`
done
echo "cmake options: ${cmake_opt[*]}"

sh -ex cicd/scripts/solve_dep_linux.sh

cd build
cmake .. ${cmake_opt[*]}
make -j8
