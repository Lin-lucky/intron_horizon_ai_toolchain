#!/usr/bin/env bash
rm /userdata/core* -rf
export LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH
./gtest_fasterrcnn --gtest_output=xml:gtestresults.xml
if [ $? -ne 0 ] ; then
    echo "failed to run gtest"
    exit 1
fi
