#!/bin/bash
# set -ex
set -eu
package_name=host_package

# install host_package
install_host_package() {  
  mkdir -p ${HOME}/.horizon/
  cp -rf ${package_name}/horizon/* ${HOME}/.horizon/
}

confirm=y
RED='\033[1;31m'
GREEN='\033[1;32m'
YELLOW='\033[1;33m'

NC='\033[0m' # No Color
if [ -d "${HOME}/.horizon" ];then
  # echo "${HOME}/.horizon is exist"
  echo -e "Do you want to reinstall all files? [${GREEN}Y${NC}/${RED}n${NC}]" 
  read confirm
  if [[ ${confirm} =~ ^(y|Y|)$ ]];then
    # echo "$confirm"
    rm -rf ${HOME}/.horizon
    install_host_package
  elif [ ${confirm} == "n" -o ${confirm} == "N" ];then
    echo -e "${RED}The file in the host_package directory are not installed in the ${HOME}/.horizon/${NC}"
    exit 1
  fi
else
  # echo "${HOME}/.horizon is not exist"
  install_host_package
fi

# install environment for toolchain sample
# export LINARO_GCC_ROOT=/opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/

FIND_STR="export HORIZON_LIB_PATH="
FIND_FILE="${HOME}/.bashrc"
if [ `grep -c "${FIND_STR}" ${FIND_FILE}` == '0' ];then
  echo -e "export HORIZON_LIB_PATH=${HOME}/.horizon/" >> ${HOME}/.bashrc
  echo "Set HORIZON_LIB_PATH environment variable value to ${HOME}/.horizon/ successfully."
else
  echo "HORIZON_LIB_PATH environment variable is exist."
fi

echo -e "${GREEN}All packages installed successfully.${NC}"

