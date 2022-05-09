#!/usr/bin/sh
DIR=./release
rm ${DIR} -rf
mkdir ${DIR} -p
cp test/test.jpg  build/bin/.
cp test ${DIR}/ -rf
cp build ${DIR}/ -rf
cp cicd/scripts/run_ut.sh ${DIR}/. 

