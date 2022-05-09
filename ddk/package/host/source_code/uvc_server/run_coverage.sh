# this script run uvc_server sample

echo "run uvc_server sample start."
export LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH
service adbd stop
/etc/init.d/usb-gadget.sh start uvc-hid

./01_transfer_nv12/01_transfer_nv12 ./01_transfer_nv12/data/1080p.nv12 1080 1920 ut

echo "run uvc_server sample end."

echo "run uvc_server_gtest start."

./bin/uvc_server_gtest

echo "run uvc_server_gtest end."
