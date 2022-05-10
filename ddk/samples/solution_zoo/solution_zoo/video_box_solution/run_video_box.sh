#!/bin/sh
usage() {
   echo "usage: sh run_body.sh "
   exit 1
}

if [ ! -L "./lib/libgomp.so.1" ];then
  ln -s /lib/libgomp.so ./lib/libgomp.so.1
  echo "create symbolic link in ./lib directory"
fi

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./lib/
vio_cfg_file=./configs/vio_config.json.x3dev.cam
vio_pipe_path="configs/vio/x3dev/"
vio_pipe_pre="iot_vio_x3_"
vio_cfg_name="./configs/vio_config.json"
vio_mode="panel_camera"

#default sensor is os8a10 
sensor="os8a10"
vio_pipe_file="${vio_pipe_path}${vio_pipe_pre}${sensor}.json"

echo userspace > /sys/devices/system/bpu/bpu0/devfreq/devfreq1/governor
echo userspace > /sys/devices/system/bpu/bpu1/devfreq/devfreq2/governor
echo 1000000000 >  /sys/devices/system/bpu/bpu0/devfreq/devfreq1/userspace/set_freq
echo 1000000000 >  /sys/devices/system/bpu/bpu1/devfreq/devfreq2/userspace/set_freqecho 105000 >/sys/devices/virtual/thermal/thermal_zone0/trip_point_1_temp
echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor

visual_cfg_func() {
  layer=$1
  width=$2
  height=$3
  layer_720p=$4
  layer_1080p=$5
  layer_2160p=$6
  sed -i 's#\("layer": \).*#\1'$layer',#g' configs/visualplugin_face.json
  sed -i 's#\("layer": \).*#\1'$layer',#g' configs/visualplugin_body.json
  sed -i 's#\("layer": \).*#\1'$layer',#g' configs/visualplugin_multisource.json
  sed -i 's#\("image_width": \).*#\1'$width',#g' configs/visualplugin_face.json
  sed -i 's#\("image_width": \).*#\1'$width',#g' configs/visualplugin_body.json
  sed -i 's#\("image_width": \).*#\1'$width',#g' configs/visualplugin_multisource.json
  sed -i 's#\("image_height": \).*#\1'$height',#g' configs/visualplugin_face.json
  sed -i 's#\("image_height": \).*#\1'$height',#g' configs/visualplugin_body.json
  sed -i 's#\("image_height": \).*#\1'$height',#g' configs/visualplugin_multisource.json
  sed -i 's#\("720p_layer": \).*#\1'$layer_720p',#g' configs/visualplugin_body.json
  sed -i 's#\("1080p_layer": \).*#\1'$layer_1080p',#g' configs/visualplugin_body.json
  sed -i 's#\("2160p_layer": \).*#\1'$layer_2160p',#g' configs/visualplugin_body.json
}

solution_cfg_func() {
  resolution=$1
  cp -rf body_solution/configs/box_filter_config_${resolution}M.json  body_solution/configs/box_filter_config.json
  cp -rf body_solution/configs/gesture_multitask_${resolution}M.json  body_solution/configs/gesture_multitask.json
  cp -rf body_solution/configs/multitask_config_${resolution}M.json  body_solution/configs/multitask_config.json
  cp -rf body_solution/configs/multitask_with_hand_${resolution}M.json  body_solution/configs/multitask_with_hand.json
  cp -rf body_solution/configs/multitask_with_hand_960x544_${resolution}M.json  body_solution/configs/multitask_with_hand_960x544.json
  cp -rf body_solution/configs/multitask_body_kps_config_${resolution}M.json  body_solution/configs/multitask_body_kps_config.json
  cp -rf body_solution/configs/segmentation_multitask_${resolution}M.json  body_solution/configs/segmentation_multitask.json
  cp -rf face_solution/configs/filter_config_${resolution}M.json face_solution/configs/filter_config.json
  cp -rf face_solution/configs/face_pose_lmk_${resolution}M.json face_solution/configs/face_pose_lmk.json
  cp -rf face_solution/configs/predict_face_pose_lmk_${resolution}M.json face_solution/configs/predict_face_pose_lmk.json
  cp -rf yolov3_solution/configs/yolov3_predict_method_${resolution}M.json yolov3_solution/configs/yolov3_predict_method.json
  cp -rf yolov3_solution/configs/yolov3_post_process_method_${resolution}M.json yolov3_solution/configs/yolov3_post_process_method.json
}

set_data_source_func() {
  vio_cfg_file=$1
  vio_mode=$2
  data_source_num=$3
  for i in $(seq 1 $data_source_num); do
    data_src_line=`cat -n ${vio_cfg_file} | grep -w "data_source" | awk '{print $1}' | sed -n ''$i'p'`
    #echo "i:$i vio_cfg_file:$vio_cfg_file data_src_line:$data_src_line vio_mode:$vio_mode"
    sed -i ''${data_src_line}'s#\("data_source": \).*#\1"'${vio_mode}'",#g' ${vio_cfg_file}
  done
}

set_cam_pipe_file_func() {
  vio_mode=$1
  sensor=$2
  vio_pipe_file=${vio_pipe_path}${vio_pipe_pre}${sensor}.json
  echo "vio_mode: $vio_mode"
  echo "vio_pipe_file: $vio_pipe_file"
  sed -i 's#\("'${vio_mode}'": \).*#\1"'${vio_pipe_file}'",#g' $vio_cfg_file
}

sensor_setting_func() {
  sensor=$1
  vio_pipe_file=$2
  param_num=$#
  if [ $param_num -eq 3 ];then
    # set defalut sensor setting
    if [ $sensor == "os8a10" -o $sensor == "os8a10_1080p" ]; then
      sed -i 's#\("i2c_bus": \).*#\1'2',#g' $vio_pipe_file
    fi
  fi
}

sensor_cfg_func() {
  local sensor=$1
  if [ $sensor == "os8a10" -o $sensor == "os8a10_1080p" ]; then
    if [ $sensor == "os8a10" ];then
      echo "sensor is os8a10, default resolution 8M, 1080P X3 JPEG Codec..."
      visual_cfg_func 4 1920 1080 5 4 0
      solution_cfg_func 8
    elif [ $sensor == "os8a10_1080p" ];then
      echo "sensor is os8a10_1080p, default resolution 1080P, 1080P X3 JPEG Codec..."
      visual_cfg_func 0 1920 1080 -1 0 -1
      solution_cfg_func 2
    fi
    echo start > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
    echo device > /sys/devices/platform/soc/b2000000.usb/b2000000.dwc3/role
    echo soc:usb-id > /sys/bus/platform/drivers/extcon-usb-gpio/unbind
    service adbd stop
    /etc/init.d/usb-gadget.sh start uvc-hid
    echo 0 > /proc/sys/kernel/printk
    echo 0xc0120000 > /sys/bus/platform/drivers/ddr_monitor/axibus_ctrl/all
    echo 0x03120000 > /sys/bus/platform/drivers/ddr_monitor/read_qos_ctrl/all
    echo 0x03120000 > /sys/bus/platform/drivers/ddr_monitor/write_qos_ctrl/all

    # reset os8a10 mipi cam
    echo 111 > /sys/class/gpio/export
    echo out > /sys/class/gpio/gpio111/direction
    echo 0 > /sys/class/gpio/gpio111/value
    sleep 0.2
    echo 1 > /sys/class/gpio/gpio111/value
    # enable mclk output to os8a10 sensor in x3sdb(mipihost0)
    echo 1 > /sys/class/vps/mipi_host0/param/snrclk_en
    echo 24000000 > /sys/class/vps/mipi_host0/param/snrclk_freq
    # enable mclk output to os8a10 sensor in x3sdb(mipihost1)
    echo 1 > /sys/class/vps/mipi_host1/param/snrclk_en
    echo 24000000 > /sys/class/vps/mipi_host1/param/snrclk_freq
  elif [ $sensor == "usb_cam_1080p" ]; then
    echo "usb_cam start, default resolution 1080P..."
    echo host > /sys/devices/platform/soc/b2000000.usb/b2000000.dwc3/role
    service adbd stop
    solution_cfg_func 2
    visual_cfg_func 0 1920 1080 -1 0 -1
  elif [ $sensor == "f37_1080p" ]; then
    echo "sensor is f37_1080p, default resolution 1080P, 1080P X3 JPEG Codec..."
    visual_cfg_func 0 1920 1080 -1 0 -1
    solution_cfg_func 2

    # reset f37_1080p mipi cam
    echo 119 > /sys/class/gpio/export
    echo out > /sys/class/gpio/gpio119/direction
    echo 0 > /sys/class/gpio/gpio119/value
    sleep 0.2
    echo 1 > /sys/class/gpio/gpio119/value
    # enable mclk output to f37_1080p sensor in x3sdb(mipihost0)
    echo 1 > /sys/class/vps/mipi_host0/param/snrclk_en
    echo 24000000 > /sys/class/vps/mipi_host0/param/snrclk_freq
    # enable mclk output to os8a10 sensor in x3sdb(mipihost1)
    echo 1 > /sys/class/vps/mipi_host1/param/snrclk_en
    echo 24000000 > /sys/class/vps/mipi_host1/param/snrclk_freq

    # support uvc device
    echo start > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
    echo device > /sys/devices/platform/soc/b2000000.usb/b2000000.dwc3/role
    echo soc:usb-id > /sys/bus/platform/drivers/extcon-usb-gpio/unbind
    service adbd stop
    /etc/init.d/usb-gadget.sh start uvc-hid
  else
    echo "error! sensor" $sensor "is not supported"
  fi
}

choose_x3_single_cam_func() {
  vio_cfg_file=$1
  vio_mode="panel_camera"
  data_source_num=1
  echo -e 'Choose lunch single cam sensor menu...pick a combo:'
  echo -e '\t1. single camera: os8a10, 2160P'
  echo -e '\t2. single camera: os8a10, 1080P'
  echo -e '\t3. single camera: usb_cam, 1080P'
  echo -e '\t4. single camera: f37_1080p, 1080P'
  echo -e 'Which would you like? '
  set_data_source_func ${vio_cfg_file} ${vio_mode} ${data_source_num}
  read aNum
  case $aNum in
    1)  echo -e "\033[33m You choose 1:os8a10, 2160p \033[0m"
      sensor=os8a10
      set_cam_pipe_file_func $vio_mode $sensor
      sensor_setting_func $sensor ${vio_pipe_file}
      sensor_cfg_func $sensor
      ;;
    2)  echo -e "\033[33m You choose 2:os8a10, 1080p \033[0m"
      sensor=os8a10_1080p
      set_cam_pipe_file_func $vio_mode $sensor
      sensor_setting_func $sensor ${vio_pipe_file}
      sensor_cfg_func $sensor
      ;;
    3)  echo -e "\033[33m You choose 3:usb_cam \033[0m"
      vio_mode="usb_cam"
      sensor=usb_cam_1080p
      set_data_source_func ${vio_cfg_file} ${vio_mode} ${data_source_num}
      set_cam_pipe_file_func $vio_mode $sensor
      sensor_cfg_func $sensor
      ;;
    4)  echo -e "\033[33m You choose 4:f37_1080p \033[0m"
      sensor=f37_1080p
      set_cam_pipe_file_func $vio_mode $sensor
      sensor_setting_func $sensor ${vio_pipe_file}
      sensor_cfg_func  $sensor
      ;;
    *) echo -e "\033[31m You choose unsupported single cam mode \033[0m"
      exit
      ;;
  esac
}

choose_x3_viotype_func() {
  vio_cfg_file=${vio_cfg_name}.x3dev.cam
  choose_x3_single_cam_func ${vio_cfg_file}
}

choose_solution_func() {
  log_level="w" #(d/i/w/e)
  sleep 3
  ./video_box/video_box ./video_box/configs/video_box_config.json ./video_box/configs/visualplugin_video_box.json ./video_box/configs/ware_config.json -${log_level} normal
}

choose_solution_func
