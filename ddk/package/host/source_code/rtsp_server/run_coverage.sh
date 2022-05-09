#!/bin/sh
# this script run rtsp_server sample and rtsp_server_gtest
echo "run rtsp_server sample start."
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./lib
./bin/rtsp_server_example ./configs/rtsp_server.json -d ut
echo "run rtsp_server sample end."
echo "run rtsp_server_gtest start"
./bin/rtsp_server_gtest
echo "run rtsp_server_gtest end"
