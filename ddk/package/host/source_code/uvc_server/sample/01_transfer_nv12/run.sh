#!/bin/sh
usage() {
   echo "usage: sh run.sh [ ut normal ]"
   exit 1
}
run_mode=$1
if [ -z $run_mode ];then
  echo "set default mode: ut"
  run_mode="ut"
fi
export LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH

service adbd stop
/etc/init.d/usb-gadget.sh start uvc-hid

./bin/01_transfer_nv12 ./data/1080p.nv12 1080 1920 $run_mode