#/bin/sh -ex

cd build/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./
./auto_test --gtest_output=xml:gtestresults.xml
cp gtestresults.xml ../../.
cd ..

