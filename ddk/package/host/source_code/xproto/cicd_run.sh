export LD_LIBRARY_PATH=./:../
cd output
cd test
./xproto_unit_test
cd -

cd tutorials/stage1_hello_world/
./stage1_hello_world
cd -

cd tutorials/stage2_max_queue_size/
./stage2_max_queue_size
cd -

cd tutorials/stage3_timeout_alarm/
./stage3_timeout_alarm
cd -