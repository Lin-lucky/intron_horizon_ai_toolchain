#!/bin/sh
usage() {
   echo "usage: sh run_body.sh "
   exit 1
}

chmod +x start_nginx.sh
sh start_nginx.sh

if [ ! -L "./lib/libgomp.so.1" ];then
  ln -s /lib/libgomp.so ./lib/libgomp.so.1
  echo "create symbolic link in ./lib directory"
fi

export LD_LIBRARY_PATH=./lib:./lib/ffmpeg:./lib/sensor:$LD_LIBRARY_PATH

#default sensor is os8a10
platform="x3dev"
sensor="os8a10"
fb_res="1080_fb"
fb_mode="jpeg_image_list"

echo userspace > /sys/devices/system/bpu/bpu0/devfreq/devfreq1/governor
echo userspace > /sys/devices/system/bpu/bpu1/devfreq/devfreq2/governor
echo 1000000000 >  /sys/devices/system/bpu/bpu0/devfreq/devfreq1/userspace/set_freq
echo 1000000000 >  /sys/devices/system/bpu/bpu1/devfreq/devfreq2/userspace/set_freq
echo 105000 >/sys/devices/virtual/thermal/thermal_zone0/trip_point_1_temp
echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
# delete fifo_id
rm grp_id_fifo jpu_id_fifo

visual_cfg_func() {
  layer=$1
  width=$2
  height=$3
  layer_720p=$4
  layer_1080p=$5
  layer_2160p=$6
  sed -i 's#\("layer": \).*#\1'$layer',#g' configs/visualplugin_face.json
  sed -i 's#\("image_width": \).*#\1'$width',#g' configs/visualplugin_face.json
  sed -i 's#\("image_height": \).*#\1'$height',#g' configs/visualplugin_face.json
  sed -i 's#\("720p_layer": \).*#\1'$layer_720p',#g' configs/visualplugin_face.json
  sed -i 's#\("1080p_layer": \).*#\1'$layer_1080p',#g' configs/visualplugin_face.json
  sed -i 's#\("2160p_layer": \).*#\1'$layer_2160p',#g' configs/visualplugin_face.json

  sed -i 's#\("layer": \).*#\1'$layer',#g' configs/web_display_plugin.json
  sed -i 's#\("image_width": \).*#\1'$width',#g' configs/web_display_plugin.json
  sed -i 's#\("image_height": \).*#\1'$height',#g' configs/web_display_plugin.json
  sed -i 's#\("720p_layer": \).*#\1'$layer_720p',#g' configs/web_display_plugin.json
  sed -i 's#\("1080p_layer": \).*#\1'$layer_1080p',#g' configs/web_display_plugin.json
  sed -i 's#\("2160p_layer": \).*#\1'$layer_2160p',#g' configs/web_display_plugin.json

  sed -i 's#\("layer": \).*#\1'$layer',#g' configs/uvc_display_plugin.json
  sed -i 's#\("image_width": \).*#\1'$width',#g' configs/uvc_display_plugin.json
  sed -i 's#\("image_height": \).*#\1'$height',#g' configs/uvc_display_plugin.json
  sed -i 's#\("720p_layer": \).*#\1'$layer_720p',#g' configs/uvc_display_plugin.json
  sed -i 's#\("1080p_layer": \).*#\1'$layer_1080p',#g' configs/uvc_display_plugin.json
  sed -i 's#\("2160p_layer": \).*#\1'$layer_2160p',#g' configs/uvc_display_plugin.json

  sed -i 's#\("layer": \).*#\1'$layer',#g' configs/rtsp_plugin.json
  sed -i 's#\("image_width": \).*#\1'$width',#g' configs/rtsp_plugin.json
  sed -i 's#\("image_height": \).*#\1'$height',#g' configs/rtsp_plugin.json
}

solution_cfg_func() {
  resolution=$1
  cp -rf multitask_perception/configs/infer_multitask_config_${resolution}M.json  multitask_perception/configs/infer_multitask_config.json
}

index_setting_func() {
  config_index=$1
  vio_cfg_file=$2
  sed -i 's#\("config_index": \).*#\1'${config_index}',#g' $vio_cfg_file
}

j3_sensor_cfg_func() {
  local sensor=$1
  if [ $sensor == "ov10635_yuv_720p" ];then
    echo "sensor is ov10635, default resolution 720P, 720P J3 JPEG Codec..."
    visual_cfg_func 0 1280 720 0 -1 -1
    solution_cfg_func 1
  elif [ $sensor = "ar0233_raw12_pwl_1080p" ];then
    echo "sensor is ar0233, default resolution 1080P, 1080P J3 JPEG Codec..."
    visual_cfg_func 0 1920 1080 -1 0 -1
    solution_cfg_func 2
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
  elif [ $sensor == "imx586_2160p" -o $sensor == "imx586_1080p" ]; then
    if [ $sensor == "imx586_2160p" ];then
      echo "sensor is imx586_2160p, default resolution 8M, 1080P X3 JPEG Codec..."
      visual_cfg_func 4 1920 1080 5 4 0
      solution_cfg_func 8
    elif [ $sensor == "imx586_1080p" ];then
      echo "sensor is imx586_1080p, default resolution 1080P, 1080P X3 JPEG Codec..."
      visual_cfg_func 0 1920 1080 -1 0 -1
      solution_cfg_func 2
    fi
    echo start > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
    # usb device mode
    # usb_mode_switch 1
    echo 0 > /proc/sys/kernel/printk
    echo 0xc0120000 > /sys/bus/platform/drivers/ddr_monitor/axibus_ctrl/all
    echo 0x03120000 > /sys/bus/platform/drivers/ddr_monitor/read_qos_ctrl/all
    echo 0x03120000 > /sys/bus/platform/drivers/ddr_monitor/write_qos_ctrl/all

    # enable mclk output to imx586 sensor in x3sdb(mipihost0)
    echo 1 > /sys/class/vps/mipi_host0/param/snrclk_en
    echo 24000000 > /sys/class/vps/mipi_host0/param/snrclk_freq
    # enable mclk output to imx586 sensor in x3sdb(mipihost1)
    echo 1 > /sys/class/vps/mipi_host1/param/snrclk_en
    echo 24000000 > /sys/class/vps/mipi_host1/param/snrclk_freq

    # reset imx586 mipi cam
    # 111 pin is reset mode
    echo 111 > /sys/class/gpio/export
    echo out > /sys/class/gpio/gpio111/direction
    echo 1 > /sys/class/gpio/gpio111/value
    # 112 pin is standby mode
    # echo 112 > /sys/class/gpio/export
    # echo out > /sys/class/gpio/gpio112/direction
    # echo 1 > /sys/class/gpio/gpio112/value
  else
    echo "error! sensor" $sensor "is not supported"
  fi
}

fb_cfg_func() {
  platform=$1
  fb_res=$2
  vio_cfg_file=$3
  echo "fb_cfg_func::platform is $platform"
  echo "fb_cfg_func::fb_res is $fb_res"
  echo "fb_cfg_func::vio_cfg_file is $vio_cfg_file"
  echo device > /sys/devices/platform/soc/b2000000.usb/b2000000.dwc3/role
  echo soc:usb-id > /sys/bus/platform/drivers/extcon-usb-gpio/unbind
  service adbd stop
  /etc/init.d/usb-gadget.sh start uvc-hid
  echo "after start uvc"
  if [ $fb_res == "1080_fb" ];then
    echo "resolution 1080P feedback start, 1080P X3 JPEG Codec..."
    solution_cfg_func 2
    visual_cfg_func 0 1920 1080 -1 0 -1
  elif [ $fb_res == "2160_fb" ];then
    echo "resolution 2160P feedback start, 1080P X3 JPEG Codec..."
    solution_cfg_func 8
    visual_cfg_func 4 1920 1080 5 4 0
  fi
}

choose_rtsp_res_func() {
  echo -e 'Choose lunch fb resolution menu...pick a combo:'
  echo -e '\t1. 1080p rtsp client '
  echo -e 'Which would you like? '
  read aNum
  case $aNum in
    1)  echo -e "\033[33m You choose 1:1080p rtsp \033[0m"
      fb_res=1080_fb
      ;;
    *) echo -e "\033[31m You choose unsupported rtsp resolution \033[0m"
      exit
      ;;
  esac
  fb_cfg_func ${platform} ${fb_res} ${vio_cfg_file}
}

choose_fb_res_func() {
  echo -e 'Choose lunch fb resolution menu...pick a combo:'
  echo -e '\t1. 1080p image list feedback'
  echo -e '\t2. 2160p image list feedback'
  echo -e '\t3. 1080p video feedback'
  echo -e 'Which would you like? '
  read aNum
  case $aNum in
    1)  echo -e "\033[33m You choose 1:1080p feedback \033[0m"
      fb_res=1080_fb
      config_index=0
      index_setting_func ${config_index} ${vio_cfg_file}
      ;;
    2)  echo -e "\033[33m You choose 2:2160p feedback \033[0m"
      fb_res=2160_fb
      config_index=1
      index_setting_func ${config_index} ${vio_cfg_file}
      ;;
    3)  echo -e "\033[33m You choose 3:1080p video feedback \033[0m"
      fb_res=1080_fb
      config_index=2
      index_setting_func ${config_index} ${vio_cfg_file}
      ;;
    *) echo -e "\033[31m You choose unsupported feedback resolution \033[0m"
      exit
      ;;
  esac
  fb_cfg_func ${platform} ${fb_res} ${vio_cfg_file}
}

choose_x3_single_cam_func() {
  vio_cfg_file=$1
  echo -e 'Choose lunch single cam sensor menu...pick a combo:'
  echo -e '\t1. single camera: os8a10, 2160P'
  echo -e '\t2. single camera: os8a10, 1080P'
  echo -e '\t3. single camera: f37_1080p, 1080P'
  echo -e '\t4. single camera: usb_cam, 1080P'
  echo -e '\t5. single camera: imx586, 2160P'
  echo -e '\t6. single camera: imx586, 1080P'
  echo -e 'Which would you like? '
  read aNum
  case $aNum in
    1)  echo -e "\033[33m You choose 1:os8a10_2160p \033[0m"
      sensor=os8a10
      config_index=0
      index_setting_func ${config_index} ${vio_cfg_file}
      sensor_cfg_func $sensor
      ;;
    2)  echo -e "\033[33m You choose 2:os8a10_1080p \033[0m"
      sensor=os8a10_1080p
      config_index=1
      index_setting_func ${config_index} ${vio_cfg_file}
      sensor_cfg_func $sensor
      ;;
    3)  echo -e "\033[33m You choose 3:f37_1080p \033[0m"
      sensor=f37_1080p
      config_index=2
      index_setting_func ${config_index} ${vio_cfg_file}
      sensor_cfg_func  $sensor
      ;;
    4)  echo -e "\033[33m You choose 4:usb_cam_1080p \033[0m"
      vio_cfg_file=./configs/x3_video_source.json.usb_cam
      sensor=usb_cam_1080p
      config_index=0
      index_setting_func ${config_index} ${vio_cfg_file}
      sensor_cfg_func $sensor
      ;;
    5)  echo -e "\033[33m You choose 5:imx586_2160p \033[0m"
      sensor=imx586_2160p
      config_index=3
      index_setting_func ${config_index} ${vio_cfg_file}
      sensor_cfg_func $sensor
      ;;
    6)  echo -e "\033[33m You choose 6:imx586_1080p \033[0m"
      sensor=imx586_1080p
      config_index=4
      index_setting_func ${config_index} ${vio_cfg_file}
      sensor_cfg_func $sensor
      ;;
    *) echo -e "\033[31m You choose unsupported single cam mode \033[0m"
      exit
      ;;
  esac
}

choose_x3_viotype_func() {
  echo -e 'Choose lunch x3 vio type menu...pick a combo:'
  echo -e '\t1. single camera'
  echo -e '\t2. single feedback'
  echo -e '\t3. single rtsp'
  echo -e 'Which would you like? '
  read aNum
    case $aNum in
    1)  echo -e "\033[33m You choose 1:single_cam \033[0m"
      vio_cfg_file=./configs/x3_video_source.json.mipi_cam
      choose_x3_single_cam_func ${vio_cfg_file}
      ;;
    2)  echo -e "\033[33m You choose 3:single_fb \033[0m"
      vio_cfg_file=./configs/x3_video_source.json.fb
      choose_fb_res_func
      ;;
    3)  echo -e "\033[33m You choose 3:single_rtsp \033[0m"
      vio_cfg_file="./configs/x3_video_source.json.rtsp"
      choose_rtsp_res_func
      ;;
  esac
}

choose_j3_camera_func() {
  video_source_file=$1
  echo -e 'Choose lunch camera sensor menu...pick a combo:'
  echo -e '\t1. mipi camera: ov10635_yuv_720p_1pipe'
  echo -e '\t2. mipi camera: ov10635_yuv_720p_4pipes'
  echo -e '\t3. mipi camera: ar0233_raw12_pwl_1080p_1pipe'
  echo -e '\t4. mipi camera: ar0233_raw12_pwl_1080p_5pipes'
  echo -e 'Which would you like? '
  read aNum
  case $aNum in
    1)  echo -e "\033[33m You choose 1:ov10635_yuv_720p_1pipe \033[0m"
      config_index=1
      sensor=ov10635_yuv_720p
      ;;
    2)  echo -e "\033[33m You choose 2:ov10635_yuv_720p_4pipes \033[0m"
      config_index=2
      sensor=ov10635_yuv_720p
      ;;
    3)  echo -e "\033[33m You choose 3:ar0233_raw12_pwl_1080p_1pipe\033[0m"
      config_index=5
      sensor=ar0233_raw12_pwl_1080p
      ;;
    4)  echo -e "\033[33m You choose 4:ar0233_raw12_pwl_1080p_5pipes\033[0m"
      config_index=6
      sensor=ar0233_raw12_pwl_1080p
      ;;
    *) echo -e "\033[31m You choose unsupported j3 camera mode \033[0m"
      exit
      ;;
  esac
  index_setting_func ${config_index} ${video_source_file}
  j3_sensor_cfg_func $sensor
}

choose_j3_viotype_func() {
  echo -e 'Choose lunch x3 vio type menu...pick a combo:'
  echo -e '\t1. mipi camera'
  echo -e 'Which would you like? '
  read aNum
    case $aNum in
    1)  echo -e "\033[33m You choose 1:mipi_camera \033[0m"
      video_source_file="./configs/j3_video_source.json.mipi_cam"
      choose_j3_camera_func ${video_source_file}
      ;;
  esac
}

choose_platform_func() {
  echo -e 'Choose lunch platform type menu...pick a combo:'
  echo -e '\t1. x3 platform'
  echo -e '\t2. j3 platform'
  echo -e 'Which would you like? '
  if [ x"${platform}" == x"x3" ];then
    aNum=$platform_aNum
  elif [ x"${platform}" == x"j3" ];then
    aNum=$platform_aNum
  else
    read aNum
  fi
  case $aNum in
    1)  echo -e "\033[33m You choose 1:x3 platform \033[0m"
      platform="x3"
      ;;
    2)  echo -e "\033[33m You choose 2:j3 platform \033[0m"
      platform="j3"
      ;;
    *) echo -e "\033[31m You choose unsupported platform type \033[0m"
      exit
      ;;
  esac
}

choose_solution_func() {
  log_level=$1
  if [ -z "$log_level" ];then
    log_level="w" #(d/i/w/e)
  fi
  echo "log_level: $log_level"
  choose_platform_func
  if [ $platform == "x3" ];then
    choose_x3_viotype_func
  elif [ $platform == "j3" ];then
    choose_j3_viotype_func
  else
    usage
  fi
  echo "vio_cfg_file: $vio_cfg_file"
  sleep 3
  chmod +x ./multitask_perception/multitask_perception
  ./multitask_perception/multitask_perception $vio_cfg_file ./multitask_perception/configs/body_solution.json ./configs/display_config.json -${log_level} normal
}

choose_solution_func $1
