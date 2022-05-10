#! /usr/bin/sh

rm -rf build
mkdir build
chmod +x gradlew

index=1

for param in $*
do
  if [ $param == "coverage" ]
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

bash -ex cicd/scripts/solve_dep_linux.sh ${cmake_opt[*]}

if [ $? -ne 0 ] ; then
    echo 'Failed to run solve_dep_linux'
    exit 1
fi
cd build
cmake .. ${cmake_opt[*]}
make -j8
