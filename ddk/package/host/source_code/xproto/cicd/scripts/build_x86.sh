#!/usr/bin/env bash
rm build.properties.local
cp build.properties.local.x86 build.properties.local
echo "-- sh [rm old dir...]"
rm -rf build
rm -rf output
rm -rf *.tmp
rm -rf .gradle
chmod +x gradlew
mkdir build
bash ./cicd/scripts/pre_env.sh
if [[ $? -ne 0 ]] ; then
    echo 'Failed to run pre_env.sh'
    exit -1
fi
cd build
cmake ..
make -j8