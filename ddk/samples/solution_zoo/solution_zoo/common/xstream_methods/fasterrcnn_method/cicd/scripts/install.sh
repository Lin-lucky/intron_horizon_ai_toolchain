#! /usr/bin/sh

rm -rf output
echo "build release version"
bash -ex ./cicd/scripts/build_linux.sh
cd build
make install
cd -
echo "build debug version"
bash -ex ./cicd/scripts/build_linux.sh debug
cd build
make install
make upload
