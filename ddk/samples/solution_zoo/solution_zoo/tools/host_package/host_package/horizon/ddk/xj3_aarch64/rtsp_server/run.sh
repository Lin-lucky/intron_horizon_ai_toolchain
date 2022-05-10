#!/bin/sh
usage() {
   echo "usage: sh run.sh [-i/-d/-w/-f]"
   exit 1
}

log_level=$1
if [ -z $log_level ];then
  echo "set default log: [-w]"
  log_level="w"
fi

run_mode=$2

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./lib
./bin/rtsp_server_example ./configs/rtsp_server.json -${log_level} ${run_mode}
