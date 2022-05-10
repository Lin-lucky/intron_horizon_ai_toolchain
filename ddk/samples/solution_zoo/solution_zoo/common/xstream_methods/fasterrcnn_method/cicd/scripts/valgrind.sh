#!/bin/sh

#usage: <command>
if [ $# != 2 ] ; then
echo "USAGE: $0 executable_program log_file"
echo " e.g.: $0 ./bin/basic valgrind.log"
exit 1;
fi

TEST_EXE=$1
VALGRIND_RESULT_FILE=$2
VALGRIND_OPTS="--leak-check=full --log-file=$VALGRIND_RESULT_FILE"
valgrind $VALGRIND_OPTS $TEST_EXE
