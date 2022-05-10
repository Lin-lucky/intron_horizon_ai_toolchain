#!/usr/bin/sh
DIR=./release
rm ${DIR} -rf
mkdir ${DIR} -p
cp configs ${DIR}/ -rf
cp vio_config/* ${DIR}/configs/ -rf
cp models ${DIR}/ -rf
cp test ${DIR}/ -rf
cp lib ${DIR}/lib/ -rf
cp build/bin/gtest_fasterrcnn ${DIR}/ -rf
cp build/bin/FasterRCNNMethod_example ${DIR}/ -rf
cp cicd/scripts/test_*.sh ${DIR} -rf
