export LD_LIBRARY_PATH=./:../
cd output
cd test
./method_init_test
./timer_test
./xstream_unit_test
./xstream_test
./node_test
./workflow_test
./xstream_threadmodel_test
./xstream_threadorder_test
./xstream_threadsafe_test
./xstream_multisource_test
./config_test
./xstream_thread_priority_test
./xstream_thread_manager_test
./xstream_xthread_pool_test
./xstream_xthread_test
cd -

cd tutorials/stage1_hello_world/
./stage1_hello_world  config/bbox_workflow.json  async
./stage1_hello_world  config/bbox_workflow.json  sync
cd -

cd tutorials/stage2_multithread/
./stage2_multithread_example  config/multithread_workflow.json
cd -

cd tutorials/stage3_update_parameter/
./stage3_update_parameter_example  config/filter_workflow.json
cd -

cd tutorials/stage4_multisource/
./stage4_multisource  config/multisource_test.json
cd -

cd tutorials/stage5_timeout_alarm/
./stage5_timeout_alarm_sync_example  config/workflow.json
./stage5_timeout_alarm_sync_example  config/workflow_node_timeout.json
cd -

cd tutorials/stage6_profile/
./stage6_profile
cd -

cd tutorials/stage7_node_output/
./stage7_node_output
cd -

cd tutorials/stage8_sub_workflow/
./stage8_sub_workflow  config/sub_workflow_config.json
cd -

cd tutorials/stage9_disable_method/
./stage9_disable_method_example  config/control_workflow.json
./stage9_pass_through_method_example  config/control_workflow.json
./stage9_use_predefined_method_example config/control_workflow.json
./stage9_best_effort_pass_through_method_example config/control_workflow_best_effort_pass_through.json
./stage9_best_effort_pass_through_method_example_2  config/control_workflow_best_effort_pass_through_2nd.json
cd -


