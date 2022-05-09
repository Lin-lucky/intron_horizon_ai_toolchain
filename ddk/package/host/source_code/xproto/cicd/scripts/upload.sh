#!/usr/bin/env bash
rm -rf output

cd build || exit -1

make install

make upload