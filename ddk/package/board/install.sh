#!/bin/bash
set -e
SCRIPTS_DIR=$(cd `dirname $0`; pwd)

RED='\033[1;31m'
GREEN='\033[1;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

if [ -z $1 ]; then
  echo "Usage: sh install.sh \${board_ip}"
  exit
fi
BOARD_IP=$1
BOARD_ACCOUNT=root
BOARD_DIR=/userdata/.horizon/

# Login
ssh ${BOARD_ACCOUNT}@${BOARD_IP} "mount -o rw,remount /"
ssh ${BOARD_ACCOUNT}@${BOARD_IP} "mkdir -p ${BOARD_DIR}"
echo "ssh login x3"

chmod u+x ${SCRIPTS_DIR}/hrt_bin_dump
scp -q ${SCRIPTS_DIR}/hrt_bin_dump ${BOARD_ACCOUNT}@${BOARD_IP}:${BOARD_DIR}/hrt_bin_dump
echo -e "hrt_bin_dump is ${GREEN}successfully${NC} installed in the ${BOARD_DIR} of the board."
chmod u+x ${SCRIPTS_DIR}/hrt_model_exec
scp -q ${SCRIPTS_DIR}/hrt_model_exec ${BOARD_ACCOUNT}@${BOARD_IP}:${BOARD_DIR}/hrt_model_exec
echo -e "hrt_model_exec is ${GREEN}successfully${NC} installed in the ${BOARD_DIR} of the board."

# install f37 sensor lib
scp -q ${SCRIPTS_DIR}/lib/f37/libf37.so ${BOARD_ACCOUNT}@${BOARD_IP}:/lib/sensorlib/
echo -e "libf37.so is ${GREEN}successfully${NC} installed in the /lib/sensorlib/ of the board."
scp -q ${SCRIPTS_DIR}/lib/f37/libjxf37_linear.so ${BOARD_ACCOUNT}@${BOARD_IP}:/etc/cam/
echo -e "libjxf37_linear.so is ${GREEN}successfully${NC} installed in the /etc/cam/ of the board."

# install imx586 sensor lib
scp -q ${SCRIPTS_DIR}/lib/imx586/libimx586.so ${BOARD_ACCOUNT}@${BOARD_IP}:/lib/sensorlib/
echo -e "libimx586.so is ${GREEN}successfully${NC} installed in the /lib/sensorlib/ of the board."
scp -q ${SCRIPTS_DIR}/lib/imx586/imx586_linear.so ${BOARD_ACCOUNT}@${BOARD_IP}:/etc/cam/
echo -e "imx586_linear.so is ${GREEN}successfully${NC} installed in the /etc/cam/ of the board."


chmod u+x ${SCRIPTS_DIR}/ai_express_webservice_display/sbin/nginx
scp -r -q ${SCRIPTS_DIR}/ai_express_webservice_display ${BOARD_ACCOUNT}@${BOARD_IP}:${BOARD_DIR}
echo -e "ai_express_webservice_display is ${GREEN}successfully${NC} installed in the ${BOARD_DIR} of the board."
ssh ${BOARD_ACCOUNT}@${BOARD_IP} "chmod u+x /userdata/.horizon/ai_express_webservice_display/sbin/nginx"

mkdir -p .install_board
cd .install_board || exit
scp -q ${BOARD_ACCOUNT}@${BOARD_IP}:/etc/profile .
sed '/#Horizon Open Explorer ENV/, /#Horizon Open Explorer ENV/d' profile > new_profile
echo -e \#Horizon Open Explorer ENV >> new_profile
echo -e export PATH=\$PATH:/usr/local/bin:/usr/bin:/bin:/usr/local/sbin:/usr/sbin:/sbin:/userdata/.horizon/:/userdata/.horizon/ai_express_webservice_display/sbin/ >> new_profile
echo -e export HORIZON_APP_PATH=\$HORIZON_APP_PATH:/userdata/.horizon/ >> new_profile
echo -e \#Horizon Open Explorer ENV >> new_profile
scp -q new_profile ${BOARD_ACCOUNT}@${BOARD_IP}:/etc/profile
ssh ${BOARD_ACCOUNT}@${BOARD_IP} "source /etc/profile"
cd ..
rm -rf .install_board
