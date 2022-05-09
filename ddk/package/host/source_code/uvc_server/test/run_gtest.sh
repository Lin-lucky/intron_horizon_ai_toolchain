#!/bin/sh
usage() {
   echo "usage: sh run_gtest.sh"
   exit 1
}

export LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH

echo start > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
echo device > /sys/devices/platform/soc/b2000000.usb/b2000000.dwc3/role
echo soc:usb-id > /sys/bus/platform/drivers/extcon-usb-gpio/unbind
service adbd stop
/etc/init.d/usb-gadget.sh start uvc-hid

./bin/uvc_server_gtest
