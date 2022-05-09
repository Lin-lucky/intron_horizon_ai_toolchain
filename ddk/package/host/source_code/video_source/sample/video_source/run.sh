#!/bin/sh
usage() {
   echo "usage: sh run.sh [-i/-d/-w/-f] [normal/ut/loop]"
   exit 1
}

log_level=$1
if [ -z $log_level ];then
  echo "set default log: [-w]"
  log_level="w"
fi
# export LOGLEVEL=3
# echo 1 > /proc/sys/kernel/printk
export LD_LIBRARY_PATH=./lib:./lib/ffmpeg:./lib/sensor:$LD_LIBRARY_PATH
rm *.jpg *.yuv *.h264 *.h265


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

  video_source_type=$3
  if [ $video_source_type == 0 ]; then
    video_source_type_aNum=1
    platform=$4
    if [ $platform == "x3" ]; then
      platform_aNum=1
      video_source_file_aNum=$5
    elif [ $platform == "j3" ]; then 
      platform_aNum=2
      video_source_file_aNum=$5
    fi
  elif [ $video_source_type == 1 ]; then
    video_source_type_aNum=2
    image_source_file_aNum=$4
  else
    usage
  fi
  echo "video source type is ${video_source_type} !!!"
fi

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

sensor_cfg_func() {
  local sensor=$1
  if [ $sensor == "os8a10_2160p" -o $sensor == "os8a10_1080p" ]; then
    if [ $sensor == "os8a10" ];then
      echo "sensor is os8a10, default resolution 8M, 1080P X3 JPEG Codec..."
    elif [ $sensor == "os8a10_1080p" ];then
      echo "sensor is os8a10_1080p, default resolution 1080P, 1080P X3 JPEG Codec..."
    fi
    echo start > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
    # usb device mode
    # usb_mode_switch 1
    echo 0 > /proc/sys/kernel/printk
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
  elif [ $sensor == "imx586_2160p" -o $sensor == "imx586_1080p" ]; then
    if [ $sensor == "imx586_2160p" ];then
      echo "sensor is imx586_2160p, default resolution 8M, 1080P X3 JPEG Codec..."
    elif [ $sensor == "imx586_1080p" ];then
      echo "sensor is imx586_1080p, default resolution 1080P, 1080P X3 JPEG Codec..."
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

function choose_j3_video_source_file() {
  echo -e 'Choose lunch single camera sensor menu...pick a combo:'
  echo -e '\t1. mipi camera: f37_raw10_1080p'
  echo -e '\t2. mipi camera: ov10635_yuv_720p'
  echo -e '\t3. mipi camera: ar0233_raw12_pwl_2048x1280'
  echo -e 'Which would you like? '
  if [ x"$run_mode" == x"normal" ];then
    aNum=$video_source_file_aNum
  elif [ x"$run_mode" == x"ut" ];then
    aNum=$video_source_file_aNum
  elif [ x"$run_mode" == x"loop"  ];then
    aNum=$video_source_file_aNum
  else
    read aNum
  fi
  case $aNum in
    1)  echo -e "\033[33m You choose 1:f37_raw10_1080p \033[0m"
      sensor=f37_1080p
      sensor_cfg_func $sensor
      video_source_file="./configs/video_source/j3dev/j3_mipi_cam_f37_raw10_1080p_offline_1pipe.json";
      ;;
    2)  echo -e "\033[33m You choose 2:ov10635_yuv_720p \033[0m"
      video_source_file="./configs/video_source/j3dev/j3_mipi_cam_ov10635_yuv_720p_offline_960_4pipes.json";
      ;;
    3)  echo -e "\033[33m You choose 3:ar0233_raw12_pwl_2048x1280 \033[0m"
      video_source_file="./configs/video_source/j3dev/j3_mipi_cam_ar0233_raw12_pwl_2048x1280_offline_5pipes.json";
      ;;
    *) echo -e "\033[31m You choose unsupported j3 camera mode \033[0m"
      exit
      ;;
  esac
}

function choose_x3_video_source_file() {
  echo -e 'Choose lunch video source file menu...pick a combo:'
  echo -e '\t1. single camera: os8a10_2160P'
  echo -e '\t2. single camera: os8a10_1080P'
  echo -e '\t3. single camera: f37_1080P'
  echo -e '\t4. single camera: imx586_2160P'
  echo -e '\t5. single camera: imx586_1080P'
  echo -e '\t6. single camera: imx327_1080P'
  echo -e '\t7. single camera: usb_cam_1080P'
  echo -e '\t8. single rtsp'
  echo -e '\t9. 1080p image list feedback'
  echo -e '\t10. 2160p image list feedback'
  echo -e '\t11. 1080p video feedback'
  echo -e 'Which would you like? '
  if [ x"$run_mode" == x"normal" ];then
    aNum=$video_source_file_aNum
  elif [ x"$run_mode" == x"ut" ];then
    aNum=$video_source_file_aNum
  elif [ x"$run_mode" == x"loop"  ];then
    aNum=$video_source_file_aNum
  else
    read aNum
  fi
  case $aNum in
    1)  echo -e "\033[33m You choose 1:os8a10_2160p \033[0m"
      sensor=os8a10_2160p
      sensor_cfg_func $sensor
      video_source_file="./configs/video_source/x3dev/mipi_cam/x3_mipi_cam_os8a10_2160p_chn0.json";
      ;;
    2)  echo -e "\033[33m You choose 2:os8a10_1080p \033[0m"
      sensor=os8a10_1080p
      sensor_cfg_func $sensor
      video_source_file="./configs/video_source/x3dev/mipi_cam/x3_mipi_cam_os8a10_1080p_chn0.json";
      ;;
    3)  echo -e "\033[33m You choose 3:f37_1080p \033[0m"
      sensor=f37_1080p
      sensor_cfg_func $sensor
      video_source_file="./configs/video_source/x3dev/mipi_cam/x3_mipi_cam_f37_1080p_chn0.json";
      ;;
    4)  echo -e "\033[33m You choose 4:imx586_2160p \033[0m"
      sensor=imx586_2160p
      sensor_cfg_func $sensor
      video_source_file="./configs/video_source/x3dev/mipi_cam/x3_mipi_cam_imx586_2160p_chn0.json";
      ;;
    5)  echo -e "\033[33m You choose 5:imx586_1080p \033[0m"
      sensor=imx586_1080p
      sensor_cfg_func $sensor
      video_source_file="./configs/video_source/x3dev/mipi_cam/x3_mipi_cam_imx586_1080p_chn0.json";
      ;;
    6)  echo -e "\033[33m You choose 6:imx327_1080p \033[0m"
      sensor=imx327_1080p
      sensor_cfg_func $sensor
      video_source_file="./configs/video_source/x3dev/mipi_cam/x3_mipi_cam_imx327_1080p_chn0.json";
      ;;
    7)  echo -e "\033[33m You choose 7:usb_cam_1080p \033[0m"
      sensor=usb_cam_1080p
      sensor_cfg_func $sensor
      video_source_file="./configs/video_source/x3dev/usb_cam/x3_usb_cam_1080p_chn0.json";
      ;;
    8)  echo -e "\033[33m You choose 8:single_rtsp \033[0m"
      video_source_file="./configs/video_source/x3dev/rtsp_client/x3_rtsp_client_chn0.json"
      ;;
    9)  echo -e "\033[33m You choose 9:1080p_image_list_feedback \033[0m"
      video_source_file="./configs/video_source/x3dev/feedback/x3_feedback_1080p_chn0.json";
      ;;
    10)  echo -e "\033[33m You choose 10:2160p_image_list_feedback \033[0m"
      video_source_file="./configs/video_source/x3dev/feedback/x3_feedback_2160p_chn0.json";
      ;;
    11)  echo -e "\033[33m You choose 11:1080p_video_feedback \033[0m"
      video_source_file="./configs/video_source/x3dev/feedback/x3_video_feedback_1080p_chn0.json";
      ;;
    *) echo -e "\033[31m You choose unsupported single cam mode \033[0m"
      exit
      ;;
  esac
}

function choose_image_source_file() {
  video_source_file="./configs/video_source/x3dev/feedback/x3_image_feedback.json";
}

function choose_image_name() {
  echo -e 'Choose lunch image name menu...pick a combo:'
  echo -e '\t1. 1080p jpeg image'
  echo -e '\t2. 2160p jpeg image'
  echo -e '\t3. 1080p nv12 image'
  echo -e '\t4. 2160p nv12 image'
  echo -e 'Which would you like? '
  if [ x"$run_mode" == x"normal" ];then
    aNum=$image_source_file_aNum
  elif [ x"$run_mode" == x"ut" ];then
    aNum=$image_source_file_aNum
  elif [ x"$run_mode" == x"loop"  ];then
    aNum=$image_source_file_aNum
  else
    read aNum
  fi
  case $aNum in
    1)  echo -e "\033[33m You choose 1:1080p_image \033[0m"
      image_name="./configs/video_source/x3dev/feedback/data/1080p.jpg"
      image_width=1920
      image_height=1080
      image_format=30    # 30 is JPEG, 3 is NV12
      ;;
    2)  echo -e "\033[33m You choose 2:2160p_image \033[0m"
      image_name="./configs/video_source/x3dev/feedback/data/2160p.jpg"
      image_width=3840
      image_height=2160
      image_format=30    # 30 is JPEG, 3 is NV12
      ;;
    3)  echo -e "\033[33m You choose 3:1080p \033[0m"
      image_name="./configs/video_source/x3dev/feedback/data/nv12_1080p.yuv"
      image_width=1920
      image_height=1080
      image_format=3     # 30 is JPEG, 3 is NV12
      ;;
    4)  echo -e "\033[33m You choose 4:2160p \033[0m"
      image_name="./configs/video_source/x3dev/feedback/data/nv12_2160p.yuv"
      image_width=3840
      image_height=2160
      image_format=3     # 30 is JPEG, 3 is NV12
      ;;
    *) echo -e "\033[31m You choose unsupported image name \033[0m"
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


choose_run_mode() {
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


function choose_lunch_video_source_type() {
  echo -e 'Choose lunch video source type menu...pick a combo:'
  echo -e '\t1. video source type'
  echo -e '\t2. image source type'
  echo -e 'Which would you like? '
  if [ "$video_source_type" == 0 ];then
    aNum=$video_source_type_aNum
  elif [ "$video_source_type" == 1 ];then
    aNum=$video_source_type_aNum
  else
    read aNum
  fi
  case $aNum in
    1)  echo -e "\033[33m You choose 1:video source type \033[0m"
      video_source_type=0
      choose_platform_func
      if [ $platform == "x3" ];then
        choose_x3_video_source_file
      elif [ $platform == "j3" ];then
        choose_j3_video_source_file
      else
        usage
      fi
      choose_run_mode
      ./bin/video_source_sample ${run_mode} ${video_source_type} ${video_source_file} -${log_level}
      # gdb ./bin/video_source_sample
      # set args 0 ./configs/video_source/x3dev/feedback/x3_feedback_1080p_chn0.json -i
      ;;
    2)  echo -e "\033[33m You choose 2:image source type \033[0m"
      video_source_type=1
      choose_image_source_file
      choose_image_name
      choose_run_mode
      # gdb debug
      # gdb ./bin/video_source_sample
      # set args 1 1 ./configs/video_source/x3dev/feedback/x3_image_feedback.json -i ./configs/video_source/x3dev/feedback/data/1080p.jpg 1920 1080 30
      ./bin/video_source_sample ${run_mode} ${video_source_type} ${video_source_file} -${log_level} \
        ${image_name} ${image_width} ${image_height} ${image_format}
      ;;
    *) echo -e "\033[31m You choose unsupported video source type mode \033[0m"
      exit
      ;;
  esac
}

choose_lunch_video_source_type
