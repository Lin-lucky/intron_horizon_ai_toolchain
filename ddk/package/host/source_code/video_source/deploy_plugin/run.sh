#!/bin/sh
usage() {
   echo "usage: sh run.sh [i/d/w/f] [normal/ut/loop]"
   exit 1
}

# export LOGLEVEL=3
echo 0 > /proc/sys/kernel/printk
export LD_LIBRARY_PATH=./lib:./lib/ffmpeg:./lib/sensor:$LD_LIBRARY_PATH
select_params=$1
log_level=$1
rm *.jpg *.yuv *.h264 *.h265

#default video source type is fb in ut mode
video_source_type="fb"
video_source_type_aNum=2
#default sensor is os8a10
sensor="os8a10"
fb_res="1080_fb"
fb_mode="jpeg_image_list"

if [ $# -gt 1 ]
then
  run_mode=$2
  if [ $run_mode == "normal" ]; then
    run_mode_aNum=1
  elif [ $run_mode == "ut" ]; then
    run_mode_aNum=2
  elif [ $run_mode == "loop" ]; then
    run_mode_aNum=3
  else
    usage
  fi
  echo "enter video source ${run_mode} mode!!!"

  platform=$3
  if [ $platform == "x3" ]; then
    platform_aNum=1
  elif [ $platform == "j3" ]; then
    platform_aNum=2
  else
    usage
  fi
  echo "platform type is ${platform} !!!"

  if [ -n "$4" ]; then
    type_aNum=$4
    if [ $type_aNum == 1 ]; then
      if [ -n "$5" ]; then
        sensor_aNum=$5
      fi
    elif [ $type_aNum == 2 ]; then
      if [ -n "$5" ]; then
        fb_res_aNum=$5
      fi
    elif [ $type_aNum == 3 ]; then
      echo ""
    elif [ $type_aNum == 4 ]; then
      if [ -n "$5" ]; then
        multi_cam_aNum=$5
      fi
    elif [ $type_aNum == 5 ]; then
      if [ -n "$5" ]; then
        multi_cam_aNum=$5
      fi
    elif [ $type_aNum == 6 ]; then
      if [ -n "$5" ]; then
        multi_mix_aNum=$5
      fi
    else
      echo "not support video_source_type:$type_aNum in ut mode"
      exit 1
    fi
  fi
fi

vin_vps_mode_setting_func() {
  vin_vps_mode=$1
  vin_config_file=$2
  sed -i 's#\("vin_vps_mode": \).*#\1'${vin_vps_mode}',#g' $vin_config_file
}

index_setting_func() {
  config_index=$1
  video_source_file=$2
  sed -i 's#\("config_index": \).*#\1'${config_index}',#g' $video_source_file
}

function usb_mode_switch() {
  usb_mode=$1
  if [ ${usb_mode} == 0 ];then
    # usb host mode, usb camera switch on
    echo host > /sys/devices/platform/soc/b2000000.usb/b2000000.dwc3/role
    service adbd stop
  else
    # uvc transfer mode
    echo device > /sys/devices/platform/soc/b2000000.usb/b2000000.dwc3/role
    echo soc:usb-id > /sys/bus/platform/drivers/extcon-usb-gpio/unbind
    service adbd stop
    /etc/init.d/usb-gadget.sh start uvc-hid
  fi
  sleep 3
}

choose_run_mode_func() {
  echo -e 'Choose lunch run_mode menu...pick a combo:'
  echo -e '\t1. start_stop_once_normal'
  echo -e '\t2. start_stop_once_ut'
  echo -e '\t3. start_stop_loop'
  echo -e 'Which would you like? '
  if [ x"$run_mode" == x"normal" ];then
    aNum=$run_mode_aNum
  elif [ x"${run_mode}" == x"ut" ];then
    aNum=$run_mode_aNum
  elif [ x"${run_mode}" == x"loop" ];then
    aNum=$run_mode_aNum
  else
    read aNum
  fi
  case $aNum in
    1)  echo -e "\033[33m You choose 1:run mode start_stop_once_normal \033[0m"
      run_mode="normal"
      ;;
    2)  echo -e "\033[33m You choose 2:run mode start_stop_once_ut \033[0m"
      run_mode="ut"
      ;;
    3)  echo -e "\033[33m You choose 3:run mode start_stop_loop \033[0m"
      run_mode="loop"
      ;;
    *) echo -e "\033[31m You choose unsupported run mode \033[0m"
      exit
      ;;
  esac
}

sensor_cfg_func() {
  local sensor=$1
  if [ $sensor == "os8a10_2160p" -o $sensor == "os8a10_1080p" ]; then
    if [ $sensor == "os8a10_2160p" ];then
      echo "sensor is os8a10, default resolution 8M, 1080P X3 JPEG Codec..."
    elif [ $sensor == "os8a10_1080p" ];then
      echo "sensor is os8a10_1080p, default resolution 1080P, 1080P X3 JPEG Codec..."
    fi
    echo start > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
    # usb device mode
    # usb_mode_switch 1
    echo 0xc0120000 > /sys/bus/platform/drivers/ddr_monitor/axibus_ctrl/all
    echo 0x03120000 > /sys/bus/platform/drivers/ddr_monitor/read_qos_ctrl/all
    echo 0x03120000 > /sys/bus/platform/drivers/ddr_monitor/write_qos_ctrl/all

    # enable mclk output to os8a10 sensor in x3sdb(mipihost0)
    echo 1 > /sys/class/vps/mipi_host0/param/snrclk_en
    echo 24000000 > /sys/class/vps/mipi_host0/param/snrclk_freq
    # enable mclk output to os8a10 sensor in x3sdb(mipihost1)
    echo 1 > /sys/class/vps/mipi_host1/param/snrclk_en
    echo 24000000 > /sys/class/vps/mipi_host1/param/snrclk_freq
  elif [ $sensor == "usb_cam_1080p" ]; then
    echo "usb_cam start, default resolution 1080P..."
    # usb host mode
    usb_mode_switch 0
  elif [ $sensor == "f37_1080p" ]; then
    echo "sensor is f37_1080p, default resolution 1080P, 1080P X3 JPEG Codec..."

    # enable mclk output to f37_1080p sensor in x3sdb(mipihost0)
    echo 1 > /sys/class/vps/mipi_host0/param/snrclk_en
    echo 24000000 > /sys/class/vps/mipi_host0/param/snrclk_freq
    # enable mclk output to os8a10 sensor in x3sdb(mipihost1)
    echo 1 > /sys/class/vps/mipi_host1/param/snrclk_en
    echo 24000000 > /sys/class/vps/mipi_host1/param/snrclk_freq
    # usb device mode
    # usb_mode_switch 1
    # reset f37 mipi cam
    # 119 pin is reset mode
    echo 119 > /sys/class/gpio/export
    echo out > /sys/class/gpio/gpio119/direction
    echo 1 > /sys/class/gpio/gpio119/value
  elif [ $sensor == "imx586_2160p" -o $sensor == "imx586_1080p" ]; then
    if [ $sensor == "imx586_2160p" ];then
      echo "sensor is imx586_2160p, default resolution 8M, 1080P X3 JPEG Codec..."
    elif [ $sensor == "imx586_1080p" ];then
      echo "sensor is imx586_1080p, default resolution 1080P, 1080P X3 JPEG Codec..."
    fi
    echo start > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
    # usb device mode
    # usb_mode_switch 1
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
  elif [ $sensor == "imx327_1080p" ]; then
    echo "sensor is imx327_1080p, default resolution 1080P, 1080P X3 JPEG Codec..."
    echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart
    echo 1 > /sys/class/vps/mipi_host1/param/stop_check_instart
  else
    echo "error! sensor" $sensor "is not supported"
  fi
}

choose_x3_multi_mix_func() {
  video_source_file=$1
  multi_mode=$2
  echo -e 'Choose lunch multi cam sensor menu...pick a combo:'
  echo -e '\t1. multi mix: f37 camera+image_list feedback 1080P+1080P'
  echo -e '\t2. multi mix: usb camera+rtsp client 1080P+1080P'
  echo -e 'Which would you like? '
  if [ x"$run_mode" == x"normal" ];then
    aNum=$multi_mix_aNum
  elif [ x"${run_mode}" == x"ut" ];then
    aNum=$multi_mix_aNum
  elif [ x"${run_mode}" == x"loop" ];then
    aNum=$multi_mix_aNum
  else
    read aNum
  fi
  case $aNum in
    1)  echo -e "\033[33m You choose 1:f37_cam+image_list_fb \033[0m"
      sensor1=f37_1080p
      if [ ${multi_mode} == "sync" ];then
        echo -e 'multi mix not supported sync mode, please use async mode'
        exit
      elif [ ${multi_mode} == "async" ];then
        config_index=0
      fi
      index_setting_func ${config_index} ${video_source_file}
      sensor_cfg_func $sensor1
      ;;
    2)  echo -e "\033[33m You choose 2:usb_cam+rtsp_client \033[0m"
      sensor1=usb_cam_1080p
      if [ ${multi_mode} == "sync" ];then
        echo -e 'multi mix not supported sync mode, please use async mode'
        exit
      elif [ ${multi_mode} == "async" ];then
        config_index=1
      fi
      index_setting_func ${config_index} ${video_source_file}
      sensor_cfg_func $sensor1
      ;;
    *) echo -e "\033[31m You choose unsupported single cam mode \033[0m"
      exit
      ;;
  esac
}

choose_x3_multi_cam_func() {
  video_source_file=$1
  multi_mode=$2
  echo -e 'Choose lunch multi cam sensor menu...pick a combo:'
  echo -e '\t1. multi camera: os8a10+f37 2160P+1080P'
  echo -e '\t2. multi camera: imx586+f37 2160P+1080P'
  echo -e '\t3. multi camera: imx327+imx327 1080P+1080P'
  echo -e '\t4. single camera bind: f37+bind_f37 1080P+1080P'
  echo -e 'Which would you like? '
  if [ x"$run_mode" == x"normal" ];then
    aNum=$multi_cam_aNum
  elif [ x"${run_mode}" == x"ut" ];then
    aNum=$multi_cam_aNum
  elif [ x"${run_mode}" == x"loop" ];then
    aNum=$multi_cam_aNum
  else
    read aNum
  fi
  case $aNum in
    1)  echo -e "\033[33m You choose 1:os8a10+f37 \033[0m"
      sensor1=os8a10_1080p
      sensor2=f37_1080p
      if [ ${multi_mode} == "sync" ];then
        config_index=30
      elif [ ${multi_mode} == "async" ];then
        config_index=31
      fi
      index_setting_func ${config_index} ${video_source_file}
      sensor_cfg_func $sensor1
      sensor_cfg_func $sensor2
      ;;
    2)  echo -e "\033[33m You choose 2:imx586+f37 \033[0m"
      sensor1=imx586_1080p
      sensor2=f37_1080p
      if [ ${multi_mode} == "sync" ];then
        config_index=32
      elif [ ${multi_mode} == "async" ];then
        config_index=33
      fi
      index_setting_func ${config_index} ${video_source_file}
      sensor_cfg_func $sensor1
      sensor_cfg_func $sensor2
      ;;
    3)  echo -e "\033[33m You choose 3:imx327+imx327 \033[0m"
      sensor=imx327_1080p
      if [ ${multi_mode} == "sync" ];then
        config_index=34
      elif [ ${multi_mode} == "async" ];then
        config_index=35
      fi
      index_setting_func ${config_index} ${video_source_file}
      sensor_cfg_func $sensor
      ;;
    4)  echo -e "\033[33m You choose 4:f37+bind_f37 \033[0m"
      sensor=imx327_1080p
      if [ ${multi_mode} == "sync" ];then
        echo -e 'bind to camera not supported sync mode, please use async mode'
        exit
      elif [ ${multi_mode} == "async" ];then
        config_index=36
      fi
      index_setting_func ${config_index} ${video_source_file}
      sensor_cfg_func $sensor
      ;;
    *) echo -e "\033[31m You choose unsupported single cam mode \033[0m"
      exit
      ;;
  esac
}

choose_x3_single_cam_func() {
  video_source_file=$1
  echo -e 'Choose lunch single cam sensor menu...pick a combo:'
  echo -e '\t1. single camera: os8a10, 2160P'
  echo -e '\t2. single camera: os8a10, 1080P'
  echo -e '\t3. single camera: f37_1080p, 1080P'
  echo -e '\t4. single camera: imx586, 2160P'
  echo -e '\t5. single camera: imx586, 1080P'
  echo -e '\t6. single camera: imx327, 1080P'
  echo -e '\t7. single camera: usb_cam, 1080P'
  echo -e 'Which would you like? '
  if [ x"$run_mode" == x"normal" ];then
    aNum=$sensor_aNum
  elif [ x"${run_mode}" == x"ut" ];then
    aNum=$sensor_aNum
  elif [ x"${run_mode}" == x"loop" ];then
    aNum=$sensor_aNum
  else
    read aNum
  fi
  case $aNum in
    1)  echo -e "\033[33m You choose 1:os8a10_2160p \033[0m"
      sensor=os8a10_2160p
      config_index=0
      index_setting_func ${config_index} ${video_source_file}
      sensor_cfg_func $sensor
      ;;
    2)  echo -e "\033[33m You choose 2:os8a10_1080p \033[0m"
      sensor=os8a10_1080p
      config_index=1
      index_setting_func ${config_index} ${video_source_file}
      sensor_cfg_func $sensor
      ;;
    3)  echo -e "\033[33m You choose 3:f37_1080p \033[0m"
      sensor=f37_1080p
      config_index=2
      index_setting_func ${config_index} ${video_source_file}
      sensor_cfg_func  $sensor
      ;;
    4)  echo -e "\033[33m You choose 4:imx586_2160p \033[0m"
      sensor=imx586_2160p
      config_index=3
      index_setting_func ${config_index} ${video_source_file}
      sensor_cfg_func $sensor
      ;;
    5)  echo -e "\033[33m You choose 5:imx586_1080p \033[0m"
      sensor=imx586_1080p
      config_index=4
      index_setting_func ${config_index} ${video_source_file}
      sensor_cfg_func $sensor
      ;;
    6)  echo -e "\033[33m You choose 6:imx327_1080p \033[0m"
      sensor=imx327_1080p
      config_index=5
      index_setting_func ${config_index} ${video_source_file}
      sensor_cfg_func $sensor
      ;;
    7)  echo -e "\033[33m You choose 7:usb_cam_1080p \033[0m"
      video_source_file=./configs/x3_video_source.json.usb_cam
      sensor=usb_cam_1080p
      config_index=0
      index_setting_func ${config_index} ${video_source_file}
      sensor_cfg_func $sensor
      ;;
    *) echo -e "\033[31m You choose unsupported single cam mode \033[0m"
      exit
      ;;
  esac
}

fb_cfg_func() {
  platform=$1
  fb_res=$2
  video_source_file=$3
  # usb device mode
  # usb_mode_switch 1
  if [ $fb_res == "1080_fb" ];then
    echo "resolution 1080P feedback start, 1080P X3 JPEG Codec..."
  elif [ $fb_res == "1280_fb" ];then
    echo "resolution 1280P feedback start, 1280P X3 JPEG Codec..."
  elif [ $fb_res == "2160_fb" ];then
    echo "resolution 2160P feedback start, 1080P X3 JPEG Codec..."
  fi
}

choose_fb_res_func() {
  echo -e 'Choose lunch fb resolution menu...pick a combo:'
  echo -e '\t1. 1080p image list feedback'
  echo -e '\t2. 1280p image list feedback'
  echo -e '\t3. 2160p image list feedback'
  echo -e '\t4. 1080p video feedback'
  echo -e 'Which would you like? '
  if [ x"$run_mode" == x"normal" ];then
    aNum=$fb_res_aNum
  elif [ x"${run_mode}" == x"ut" ];then
    aNum=$fb_res_aNum
  elif [ x"${run_mode}" == x"loop" ];then
    aNum=$fb_res_aNum
  else
    read aNum
  fi
  case $aNum in
    1)  echo -e "\033[33m You choose 1:1080p image list feedback \033[0m"
      fb_res=1080_fb
      config_index=0
      index_setting_func ${config_index} ${video_source_file}
      ;;
    2)  echo -e "\033[33m You choose 2:1280p image list feedback \033[0m"
      fb_res=1280_fb
      config_index=3
      index_setting_func ${config_index} ${video_source_file}
      ;;
    3)  echo -e "\033[33m You choose 3:2160p image list feedback \033[0m"
      fb_res=2160_fb
      config_index=1
      index_setting_func ${config_index} ${video_source_file}
      ;;
    4)  echo -e "\033[33m You choose 4:1080p video feedback \033[0m"
      fb_res=1080_fb
      config_index=2
      index_setting_func ${config_index} ${video_source_file}
      ;;
    *) echo -e "\033[31m You choose unsupported feedback resolution \033[0m"
      exit
      ;;
  esac
  fb_cfg_func ${platform} ${fb_res} ${video_source_file}
}

choose_x3_video_source_type_func() {
  echo -e 'Choose lunch x3 video source type menu...pick a combo:'
  echo -e '\t1. single cam'
  echo -e '\t2. single feedback'
  echo -e '\t3. single rtsp'
  echo -e '\t4. multi cam sync'
  echo -e '\t5. multi cam async'
  echo -e '\t6. multi mix async'
  echo -e 'Which would you like? '
  if [ x"$run_mode" == x"normal" ];then
    aNum=$type_aNum
  elif [ x"${run_mode}" == x"ut" ];then
    aNum=$type_aNum
  elif [ x"${run_mode}" == x"loop" ];then
    aNum=$type_aNum
  else
    read aNum
  fi
  case $aNum in
    1)  echo -e "\033[33m You choose 1:single_cam \033[0m"
      video_source_file="./configs/x3_video_source.json.mipi_cam"
      choose_x3_single_cam_func ${video_source_file}
      ;;
    2)  echo -e "\033[33m You choose 2:single_fb \033[0m"
      video_source_file="./configs/x3_video_source.json.fb"
      choose_fb_res_func
      ;;
    3)  echo -e "\033[33m You choose 3:single_rtsp \033[0m"
      video_source_file="./configs/x3_video_source.json.rtsp"
      ;;
    4)  echo -e "\033[33m You choose 4:multi_cam_sync \033[0m"
      video_source_file="./configs/x3_video_source.json.mipi_cam"
      multi_mode="sync"
      choose_x3_multi_cam_func ${video_source_file} ${multi_mode}
      ;;
    5)  echo -e "\033[33m You choose 5:multi_cam_async \033[0m"
      video_source_file="./configs/x3_video_source.json.mipi_cam"
      multi_mode="async"
      choose_x3_multi_cam_func ${video_source_file} ${multi_mode}
      ;;
    6)  echo -e "\033[33m You choose 6:multi_mix_async \033[0m"
      video_source_file="./configs/x3_video_source.json.multi_mix"
      multi_mode="async"
      choose_x3_multi_mix_func ${video_source_file} ${multi_mode}
      ;;
    *) echo -e "\033[31m You choose unsupported video source type mode \033[0m"
      exit
      ;;
  esac
}

choose_j3_camera_func() {
  video_source_file=$1
  echo -e 'Choose lunch camera sensor menu...pick a combo:'
  echo -e '\t1. mipi camera: ov10635_yuv_720p_1pipe'
  echo -e '\t2. mipi camera: ov10635_yuv_720p_4pipes'
  echo -e '\t3. mipi camera: ar0233_raw12_pwl_2048x_1280_1pipe'
  echo -e '\t4. mipi camera: ar0233_raw12_pwl_2048x_1280_5pipes'
  echo -e '\t5. mipi camera: ar0233_raw12_pwl_1080p_1pipe'
  echo -e '\t6. mipi camera: ar0233_raw12_pwl_1080p_5pipes'
  echo -e 'Which would you like? '
  if [ x"$run_mode" == x"normal" ];then
    aNum=$sensor_aNum
  elif [ x"${run_mode}" == x"ut" ];then
    aNum=$sensor_aNum
  elif [ x"${run_mode}" == x"loop" ];then
    aNum=$sensor_aNum
  else
    read aNum
  fi
  case $aNum in
    1)  echo -e "\033[33m You choose 1:ov10635_yuv_720p_1pipe \033[0m"
      config_index=1
      index_setting_func ${config_index} ${video_source_file}
      ;;
    2)  echo -e "\033[33m You choose 2:ov10635_yuv_720p_4pipes \033[0m"
      config_index=2
      index_setting_func ${config_index} ${video_source_file}
      ;;
    3)  echo -e "\033[33m You choose 3:ar0233_raw12_pwl_2048x_1280_1pipe\033[0m"
      config_index=3
      index_setting_func ${config_index} ${video_source_file}
      ;;
    4)  echo -e "\033[33m You choose 4:ar0233_raw12_pwl_2048x_1280_5pipes\033[0m"
      config_index=4
      index_setting_func ${config_index} ${video_source_file}
      ;;
    5)  echo -e "\033[33m You choose 3:ar0233_raw12_pwl_1080p_1pipe\033[0m"
      config_index=5
      index_setting_func ${config_index} ${video_source_file}
      ;;
    6)  echo -e "\033[33m You choose 4:ar0233_raw12_pwl_1080p_5pipes\033[0m"
      config_index=6
      index_setting_func ${config_index} ${video_source_file}
      ;;
    *) echo -e "\033[31m You choose unsupported j3 camera mode \033[0m"
      exit
      ;;
  esac
}

choose_j3_video_source_type_func() {
  echo -e 'Choose lunch j3 video source type menu...pick a combo:'
  echo -e '\t1. mipi camera'
  echo -e 'Which would you like? '
  if [ x"$run_mode" == x"normal" ];then
    aNum=$type_aNum
  elif [ x"${run_mode}" == x"ut" ];then
    aNum=$type_aNum
  elif [ x"${run_mode}" == x"loop" ];then
    aNum=$type_aNum
  else
    read aNum
  fi
  case $aNum in
    1)  echo -e "\033[33m You choose 1:mipi_camera \033[0m"
      video_source_file="./configs/j3_video_source.json.mipi_cam"
      choose_j3_camera_func ${video_source_file}
      ;;
    *) echo -e "\033[31m You choose unsupported video source type mode \033[0m"
      exit
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

function select_mode() {
  log_level=$1
  if [ -z "$log_level" ];then
    log_level="w"
  fi
  echo "log_level: $log_level"
  choose_platform_func
  if [ $platform == "x3" ];then
    choose_x3_video_source_type_func
  elif [ $platform == "j3" ];then
    choose_j3_video_source_type_func
  else
    usage
  fi
  choose_run_mode_func
  echo "video source file: ${video_source_file}"
  # gdb ./bin/video_source_plugin_sample
  # set args ./configs/x3_video_source.json.mipi_cam normal -i
  ./bin/video_source_plugin_sample ${video_source_file} ${run_mode} -${log_level}
}

select_mode ${log_level}
