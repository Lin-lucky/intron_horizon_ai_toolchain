#!/usr/bin/env bash
# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
SKYBLUE='\033[0;36m'
PLAIN='\033[0m'

cur_dir=$(cd $(dirname $0);pwd)
pypi_link="https://mirrors.aliyun.com/pypi/simple"
trusted_host="mirrors.aliyun.com"

cd $cur_dir || exit 33

if [ "$BASH_VERSION" = "" ]; then
    echo -e "${RED}please change sh to bash${PLAIN}"
    exit 33
else
    sh ai_toolchain/env_install.sh
    source ~/.bashrc
fi

pip_version=$(pip3 -V | awk '{print $2}' | awk -F. '{print $1}')
if [ $pip_version -lt 20 ]; then
    while true; do
        read -p "pip version too low. Do you wish to upgrade pip to the version 20.1.1? [yes|no]" yn
        case $yn in
            [Yy]* )  python3 -m pip install pip==20.1.1 -i https://mirrors.aliyun.com/pypi/simple \
                    --trusted-host  mirrors.aliyun.com \
                    --extra-index-url=http://mirrors.aliyun.com/pypi/simple/ \
                    --trusted-host mirrors.aliyun.com  --user -U;
                    if [ $? -ne 0 ];
                    then
                        echo "======================================================"
                        echo "'pip 20.1.1' install error, please install it manually"
                        echo "recommend command: "
                        echo "pip3 install pip==20.1.1   \
                        -i $pypi_link --trusted-host $trusted_host \
                        --extra-index-url=http://mirrors.aliyun.com/pypi/simple/ \
                        --trusted-host mirrors.aliyun.com  --user -U"
                        echo "======================================================"
                        exit 21
                    fi;
                    break;;
            [Nn]* )
                  echo "======================================================"
                  echo "Please manually upgrade pip to version 20 or above";
                  echo "======================================================"
                  exit 22; break;;
            * ) echo "Please answer yes or no.";;
        esac
    done
fi

cd ai_toolchain || exit

res=$(echo `pip3 list |grep x3-tc-ui` | grep "x3-tc-ui")
if [[ "$res" != "" ]]; then
  pip3 uninstall x3_tc_ui -y
fi

echo -e "${SKYBLUE}start to install dependencies...${PLAIN}"

pip3 install --user --disable-pip-version-check --quiet cython==0.28.2 -i $pypi_link --trusted-host $trusted_host

pip3 install --user --disable-pip-version-check --quiet pycocotools==2.0.00 -i $pypi_link --trusted-host $trusted_host

pip3 install --user --disable-pip-version-check --quiet absl-py==0.7.0 -i $pypi_link --trusted-host $trusted_host

pip3 install --user --disable-pip-version-check --quiet zmq==0.0.0 -i $pypi_link --trusted-host $trusted_host

pip3 install --user --disable-pip-version-check --quiet easydict==1.7 -i $pypi_link --trusted-host $trusted_host

pip3 install --user --disable-pip-version-check --quiet numpy==1.16.5 -i $pypi_link --trusted-host $trusted_host

pip3 install --user --disable-pip-version-check --quiet opencv-python==3.4.5.20 -i $pypi_link --trusted-host $trusted_host

pip3 install --user --disable-pip-version-check --quiet torchvision==0.10.1 -i $pypi_link --trusted-host $trusted_host

hbdk_whl=$(ls  | grep hbdk | grep -v hbdk_model_verifier)
if [ ! -z $hbdk_whl ]; then
  echo "Install hbdk ${hbdk_whl}..."
  pip3 install -U --user --disable-pip-version-check --quiet ${hbdk_whl} \
    -i $pypi_link --trusted-host $trusted_host
else
  echo -e "${YELLOW}WARNING!!! hbdk ${hbdk_whl} MISSING!!!${PLAIN}"
fi

hbdk_mv_whl=$(ls  | grep hbdk_model_verifier)
if [ ! -z $hbdk_mv_whl ]; then
  echo "Install hbdk model verifier ${hbdk_mv_whl}..."
  pip3 install -U --user --disable-pip-version-check --quiet ${hbdk_mv_whl} \
      -i $pypi_link --trusted-host $trusted_host
else
  echo -e "${YELLOW}WARNING!!! hbdk model verifier  ${hbdk_mv_whl} MISSING!!!${PLAIN}"
fi

nn_whl=$(ls | grep horizon_nn | grep -v gpu)
if [ ! -z $nn_whl ]; then
  echo "Install horizon_nn ${nn_whl}..."
  pip3 install -U --user --disable-pip-version-check --quiet ${nn_whl} \
      -i $pypi_link --trusted-host $trusted_host
else
  echo -e "${YELLOW}WARNING!!! horizon_nn ${nn_whl} MISSING!!!${PLAIN}"
fi

tool_whl=$(ls | grep horizon_tc_ui)
if [ ! -z $tool_whl ]; then
  echo "Install horizon_tc_ui ${tool_whl}..."
  pip3 install -U --user --disable-pip-version-check --quiet ${tool_whl} \
      -i $pypi_link --trusted-host $trusted_host
else
  echo -e "${YELLOW}WARNING!!! horizon_tc_ui ${tool_whl} MISSING!!!${PLAIN}"
fi

# 检查环境变量
paths=($(echo $PATH | tr ":" " "))
ok="false"
for p in "${paths[@]}";
do
  if [[ $p =~ $HOME/.local/bin/?$ ]]; then
    ok="true"
    break
  fi
done

if [ "${ok}" = "false" ]; then
  echo -e "export PATH=\$HOME/.local/bin:\$PATH" >> $HOME/.bash_profile
  echo "================================================================"
  echo "dependency installation finished. please make sure .bash_profile will be automatically executed during log in and RESTART the terminal!!!"
  echo "================================================================"
  exit 33
fi

echo -e "${GREEN}dependency installation finished.${PLAIN}"
echo -e "${GREEN}please make sure .bash_profile will be automatically executed during log in and RESTART the terminal!!!.${PLAIN}"
