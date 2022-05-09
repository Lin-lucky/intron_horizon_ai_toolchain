# this script run video_source_noplugin sample and gtestcase in ut mode

echo "run video_source_noplugin sample start."
# 1. run video source noplugin sample on x3 board
# 1.1 run os8a10_2160P
# 1.2 run os8a10_1080P
# 1.3 run f37_1080p
sh run.sh w ut 0 x3 3

# 1.4 run imx586_2160P
# 1.5 run imx586_1080P
# 1.6 run imx327_1080P
# 1.7 run usb_cam_1080P
# 1.8 run single rtsp
sh run.sh w ut 0 x3 8
# 1.9 run 1080p image list feedback
sh run.sh w ut 0 x3 9

# 1.10 run 2160p image list feedback
sh run.sh w ut 0 x3 10

# # 1.11 run 1080p video feedback
sh run.sh w ut 0 x3 11

# 2. run video source noplugin sample on j3 board
# 2.1 run f37_raw10_1080p
# sh run.sh w ut 0 j3 1

# 2.2 run ov10635_yuv_720p
#sh run.sh w ut 0 j3 2

# 2.3 run ar0233_raw12_pwl_2048x1280
# sh run.sh w ut 0 j3 3

# 3. run image source
# 3.1 run 1080p jpeg image
sh run.sh w ut 1 1

# 3.2 run 2160p jpeg image
sh run.sh w ut 1 2

# 3.3 run 1080p nv12 image
sh run.sh w ut 1 3

# 3.4 run 2160p nv12 image
sh run.sh w ut 1 4

echo "run video_source_noplugin sample end."

echo "run video source gtestcase start."

# 1. run video source gtestcase on x3 board
# 1.1 run os8a10_2160P
# 1.2 run os8a10_1080P
# 1.3 run f37_1080p
sh run_gtest.sh w 0 3

# 1.4 run imx586_2160P
# 1.5 run imx586_1080P
# 1.6 run imx327_1080P
# 1.7 run usb_cam_1080P
# 1.8 run single rtsp
sh run_gtest.sh w 0 8
# 1.9 run 1080p image list feedback
sh run_gtest.sh w 0 9

# 1.10 run 2160p image list feedback
sh run_gtest.sh w 0 10

# 1.11 run 1080p video feedback
sh run_gtest.sh w 0 11

# 2. run video source gtest on j3 board
# 2.1 run f37_raw10_1080p
# sh run_gtest.sh w 0 j3 1

# 2.2 run ov10635_yuv_720p
#sh run.sh w 0 j3 2

# 2.3 run ar0233_raw12_pwl_2048x1280
# sh run.sh w 0 j3 3

# 3. run image source
# 3.1 run 1080p jpeg image
sh run_gtest.sh w 1 1

# 3.2 run 2160p jpeg image
sh run_gtest.sh w 1 2

# 3.3 run 1080p nv12 image
sh run_gtest.sh w 1 3

# 3.4 run 2160p nv12 image
sh run_gtest.sh w 1 4
