#! /usr/bin/sh -ex

echo "test linux script"
export LD_LIBRARY_PATH=./lib/
echo "-------------gtest-------------"

if [ $# -lt 1 ];then
  XSTREAM_XML_NAME=xml:gtestresults.xml
else
  XSTREAM_XML_NAME=$1
fi



./bin/xstream_unit_test --gtest_output=${XSTREAM_XML_NAME}
if [ $? -ne 0 ] ; then
    echo "failed to run xstream_unit_test"
    exit 1
fi

echo "-------------test bbox_filter_example-------------"
./bin/bbox_filter_example ./config/filter.json
if [ $? -ne 0 ] ; then
    echo "failed to run bbox_filter_example test"
    exit 1
fi
echo "-------------test c_bbox_filter_example-------------"
#./bin/c_bbox_filter_example ./config/filter.json
#if [ $? -ne 0 ] ; then
#    echo "failed to run c_bbox_filter_example test"
#    exit 1
#fi
echo "-------------test profiler_test-------------"
./bin/profiler_test ./config/filter.json
if [ $? -ne 0 ] ; then
    echo "failed to run profiler_test test"
    exit 1
fi
echo "-------------profiler content-------------"
cat ./profiler.txt

echo "-------------test disable_method_test-------------"
./bin/disable_method_test ./config/filter.json
if [ $? -ne 0 ] ; then
    echo "failed to run disable_method_test test"
    exit 1
fi
echo "run all ok"
