# this script run video_source_plugin sample in ut mode

# 1. run single camera
# 1.3 run f37_1080p camera
sh run.sh w ut x3 1 3

# 2. run single video feedback
# 2.1 run single image list 1080p feedback
sh run.sh w ut x3 2 1
# 2.2 run single image list 2160p feedback
sh run.sh w ut x3 2 2
# 2.3 run single video 1080p feedback
sh run.sh w ut x3 2 2

# 3. run rtsp client
sh run.sh w ut x3 3
# 4. run multi camera sync

# 5. run multi camera async
# 5.4 run f37 camera + bind_f37
sh run.sh w ut x3 5 4

# 6. run single camera + single feedback
# 6.1 run f37 camera + image list 1080p feedback
sh run.sh w ut x3 6 1

