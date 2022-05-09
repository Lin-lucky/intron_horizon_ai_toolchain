#!/bin/bash
ddk_version=v1.9.4
SCRIPTS_DIR=$(readlink -f "$(dirname "$0")")
cd "$SCRIPTS_DIR" || exit
RED='\033[1;31m'
GREEN='\033[1;32m'
YELLOW='\033[1;33m'
SKYBLUE='\033[0;36m'
PLAIN='\033[0m'
pypi_link="https://mirrors.aliyun.com/pypi/simple"
trusted_host="mirrors.aliyun.com"

usage() {
  echo "Usage: bash install_host_package.sh [OPTIONS]"
  echo ""
  echo "Options:"
  echo "  -h, --help            show this help message and exit"
  echo "  -y                    answer yes for all questions"
}

get_sys_version(){
	if [ -f /etc/redhat-release ]; then
	    release="centos"
	elif cat /etc/issue | grep -Eqi "ubuntu"; then
	    release="ubuntu"
	elif cat /etc/issue | grep -Eqi "centos|red hat|redhat"; then
	    release="centos"
	elif cat /proc/version | grep -Eqi "ubuntu"; then
	    release="ubuntu"
	elif cat /proc/version | grep -Eqi "centos|red hat|redhat"; then
	    release="centos"
	fi

	if [ $release == "ubuntu" ]; then
		ubuntuVersion=$(awk -F'[= "]' '/VERSION_ID/{print $3}' /etc/os-release)
		if [ "$ubuntuVersion" == "18.04" ]; then
		  echo -e "System is ${GREEN}Ubuntu${PLAIN}, configuration starts..."
		else
		  echo -e "${RED}Error:${PLAIN} This script can not be run in your system now!" && exit 1
		fi
	elif [ $release == "centos" ]; then
		os_release=$(grep "CentOS" /etc/redhat-release 2>/dev/null)
		if echo "$os_release"|grep "release 7" >/dev/null 2>&1
		then
			echo -e "System is ${GREEN}CentOS${PLAIN}, configuration starts..."
		else
			echo -e "${RED}Error:${PLAIN} This script can not be run in your system now!" && exit 1
		fi
	else
		echo -e "${RED}Error:${PLAIN} This script can not be run in your system now!" && exit 1
	fi
}

check_pip() {
  if [ "$release" == "centos" ]; then
    if [ -z "$(yum list installed |grep pip)" ]; then
      echo "======================================================"
      echo "'python3-pip' not installed "
      echo "please make sure python3 is set to 'python3.6' then install 'python3-pip' manually via command:"
      echo "sudo yum install python3-pip -y"
      echo "then re execute the install_host_package script"
      echo "======================================================"
      exit 25
    else
      echo -e "${SKYBLUE}python3-pip already installed${PLAIN}"
    fi
  else
    if [ -z "$(dpkg -l | grep -E '^ii' | grep python3-pip)" ]; then
      echo "======================================================"
      echo "'python3-pip' not installed "
      echo "please make sure python3 is set to 'python3.6' then install 'python3-pip' manually via command:"
      echo "sudo apt-get install python3-pip -y"
      echo "then re execute the install_host_package script"
      echo "======================================================"
      exit 25
    else
      echo -e "${SKYBLUE}python3-pip already installed${PLAIN}"
    fi
  fi
}

install_ddk_vcs() {
  check_pip
  if [ ! -f "./ddk_vcs-0.2.1-py3-none-any.whl" ]; then
    echo -e "${RED}ddk_vcs whl missing${PLAIN}"
    exit 1
  fi
  res=$(pip3 --disable-pip-version-check list 2>&1  | grep ddk-vcs | grep "ddk-vcs")
  if [[ "$res" != "" ]]; then
    pip3 --disable-pip-version-check uninstall ddk_vcs -y
  fi
  pip3 --disable-pip-version-check install ddk_vcs-0.2.1-py3-none-any.whl -i $pypi_link --trusted-host $trusted_host
}

# install host_package
install_host_package() {
  ddk_vcs update_version -v $ddk_version
  for file in ./host_package/xj3_aarch64/*; do
    if test -f $file; then
      ddk_vcs install $file -p aarch_64
    fi
  done
  for file in ./host_package/xj3_x86_64_gcc_4.8.5/*; do
    if test -f $file; then
      ddk_vcs install $file -p x86_64_gcc4.8.5
    fi
  done
  echo -e "${GREEN}All packages installed successfully.${PLAIN}"
}

cmake_install() {
  cmake_version=$(cmake --version | grep version | awk '{print $3}')
  if [[ $cmake_version =~ 3.14.5 ]]; then
    echo -e "${SKYBLUE}cmake version 3.14.5, version correct${PLAIN}"
  else
    echo "cmake version incompatible. cmake 3.14.5 needed"
    tar -zxf cmake-3.14.5-Linux-x86_64.tar.gz
    mkdir -p ~/.horizon/cmake
    mv cmake-3.14.5-Linux-x86_64/* ~/.horizon/cmake/
    echo "======================================================"
    echo "The file has been placed in the corresponding location"
    echo "please set cmake to env manually via command:"
    echo "echo "export PATH=\$HOME/.horizon/cmake/bin:\$PATH" >> "\${HOME}"/.bashrc"
    echo "then enter bash to refresh the terminal and re execute install_host_package script"
    echo "======================================================"
    exit 64
  fi
}

runtime_ubuntu() {
  gcc_version=$(gcc --version |grep gcc |awk -F '[ ]' '{print $4}')
  if [[ $gcc_version =~ 4.8.5 ]]; then
    echo -e "${SKYBLUE}gcc version $gcc_version, version correct${PLAIN}"
  else
    echo "======================================================"
    echo "gcc version incompatible. gcc 4.8.5 needed"
    echo "Please install gcc 4.8.5 manually via command"
    echo "sudo apt-get install gcc-4.8 -y"
    echo "and configure it to be the default one"
    echo "update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 20"
    echo "then re execute the install_host_package script"
    echo "======================================================"
    exit 50
  fi
  g_plus_plus_version=$(g++ --version |grep g++ |awk -F '[ ]' '{print $4}')
  if [[ $g_plus_plus_version =~ 4.8.5 ]]; then
    echo -e "${SKYBLUE}g++ version $g_plus_plus_version, version correct${PLAIN}"
  else
    echo "======================================================"
    echo "g++ version incompatible. g++ 4.8.5 needed"
    echo "Please install g++ 4.8.5 manually via command"
    echo "sudo apt-get install g++-4.8 -y"
    echo "and configure it to be the default one"
    echo "update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 20"
    echo "then re execute the install_host_package script"
    echo "======================================================"
    exit 51
  fi
  cmake_install
}

runtime_centos() {
  gccVersion="$(yum list installed |grep ^gcc.x86_64 |awk 'NR==1{print $2}')"
  if [ "$gccVersion" == "" ]; then
    echo "======================================================"
    echo "gcc not installed "
    echo "please install 'gcc' manually via command:"
    echo "sudo yum install gcc -y "
    echo "then re execute the install_host_package script"
    echo "======================================================"
    exit 22
  else
    echo -e "${SKYBLUE}gcc installed, version is ${gccVersion}${PLAIN}"
  fi
  gPlusPlusVersion=$(yum list installed |grep gcc-c++ |awk 'NR==1{print $2}')
  if [ "$gPlusPlusVersion" == "" ]; then
    echo "======================================================"
    echo "g++ not installed "
    echo "please install 'g++' manually via command:"
    echo "sudo yum install gcc-c++ -y "
    echo "then re execute the install_host_package script"
    echo "======================================================"
    exit 22
  else
    echo -e "${SKYBLUE}g++ installed, version is ${gPlusPlusVersion}${PLAIN}"
  fi
  cmake_install
}

set_env() {
  OUTDATED_FIND_STR="export HORIZON_LIB_PATH=${HOME}/.horizon/xj3/"
  FIND_STR="export HORIZON_LIB_PATH="
  FIND_FILE="${HOME}/.bashrc"
  DDK_LIB_FIND="export DDK_LIB_PATH="
  XJ3_AARCH_FIND="export XJ3_AARCH_PATH="
  XJ3_X86_GCC485_FIND="export XJ3_X86_GCC485_PATH="
  if [ "$release" == "centos" ]; then
    runtime_centos
    if [ "$LC_ALL" == "" ] || [ "$LANG" == "" ]; then
      echo "======================================================"
      echo "environment variable 'LC_ALL' not set"
      echo "please set LC_ALL manually via command:"
      echo "echo -e "export LC_ALL=zh_CN.utf8" >>"${HOME}"/.bashrc"
      echo "echo -e "export LANG=zh_CN.utf8" >>"${HOME}"/.bashrc"
      echo "source ~/.bashrc"
      echo -e "if show this message: ${YELLOW}"bash: warning: setlocale: LC_ALL: cannot change locale"${PLAIN}"
      echo "please set the locales manually via command:"
      echo "localedef -c -f UTF-8 -i zh_CN zh_CN.utf8"
      echo "then enter bash to refresh the terminal and re execute install_host_package script"
      echo "======================================================"
      exit 31
    fi
  else
    runtime_ubuntu
    if [ "$LC_ALL" == "" ] || [ "$LANG" == "" ]; then
      echo "======================================================"
      echo "environment variable 'LC_ALL' and 'LANG' not set"
      echo "please set 'LC_ALL' and 'LANG' manually via command:"
      echo "echo -e "export LC_ALL=zh_CN.utf8" >>"${HOME}"/.bashrc"
      echo "echo -e "export LANG=zh_CN.utf8" >>"${HOME}"/.bashrc"
      echo -e "if show this message: ${YELLOW}"bash: warning: setlocale: LC_ALL: cannot change locale"${PLAIN}"
      echo "please set the locales manually via command:"
      echo "sudo dpkg-reconfigure locales"
      echo "then enter number of zh_CN.UTF-8 twice"
      echo "then enter bash to refresh the terminal and re execute install_host_package script"
      echo "======================================================"
      exit 31
    fi
  fi
  # LINARO_GCC_ROOT
  if [ "$LINARO_GCC_ROOT" == "" ]; then
    echo "======================================================"
    echo "environment variable 'LINARO_GCC_ROOT' not set "
    echo "If gcc 6.5.0(for ARM) is not installed, please download it via link or use the tarball under ddk/tools in Open Explorer: "
    echo "http://releases.linaro.org/components/toolchain/binaries/6.5-2018.12/aarch64-linux-gnu/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu.tar.xz"
    echo "Place the files under this path like this: /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/"
    echo "If gcc 6.5.0(for ARM) is already installed"
    echo "please set gcc path to LINARO_GCC_ROOT in ~/.bashrc or other proper places"
    echo "then restart terminal to proceed and re execute install_host_package script"
    echo "======================================================"
    exit 52
  else
    echo -e "${SKYBLUE}LINARO_GCC_ROOT already set${PLAIN}"
  fi
  # HORIZON_LIB_PATH
  if [ `grep -c "${FIND_STR}" ${FIND_FILE}` == '0' ];then
    echo -e "export HORIZON_LIB_PATH=${HOME}/.horizon/" >> ${HOME}/.bashrc
    echo "Set HORIZON_LIB_PATH environment variable value to ${HOME}/.horizon/ successfully."
  elif [ `grep -c "${OUTDATED_FIND_STR}" ${FIND_FILE}` != '0' ];then
    sed -i 's#HORIZON_LIB_PATH=.*#HORIZON_LIB_PATH=${HOME}/.horizon/#g' ${HOME}/.bashrc
    echo "Update HORIZON_LIB_PATH environment variable value to ${HOME}/.horizon/ successfully."
  else
    sed -i 's#HORIZON_LIB_PATH=.*#HORIZON_LIB_PATH=${HOME}/.horizon/#g' ${HOME}/.bashrc
    echo "Set HORIZON_LIB_PATH environment variable value to ${HOME}/.horizon/ successfully."
  fi

  # DDK_LIB_PATH
  if [ `grep -c "${DDK_LIB_FIND}" ${FIND_FILE}` == '0' ];then
    echo -e "export DDK_LIB_PATH=${HOME}/.horizon/ddk" >> ${HOME}/.bashrc
    echo "Set DDK_LIB_PATH environment variable value to ${HOME}/.horizon/ddk successfully."
  else
    sed -i 's#DDK_LIB_PATH=.*#DDK_LIB_PATH=${HOME}/.horizon/ddk#g' ${HOME}/.bashrc
    echo "Set DDK_LIB_PATH environment variable value to ${HOME}/.horizon/ddk successfully."
  fi

  # XJ3_AARCH_PATH
  if [ `grep -c "${XJ3_AARCH_FIND}" ${FIND_FILE}` == '0' ];then
    echo -e "export XJ3_AARCH_PATH=${HOME}/.horizon/ddk/xj3_aarch64/" >> ${HOME}/.bashrc
    echo "Set XJ3_AARCH_PATH environment variable value to ${DDK_LIB_PATH}/xj3_aarch64/ successfully."
  else
    sed -i 's#XJ3_AARCH_PATH=.*#XJ3_AARCH_PATH=${HOME}/.horizon/ddk/xj3_aarch64/#g' ${HOME}/.bashrc
    echo "Set XJ3_AARCH_PATH environment variable value to ${DDK_LIB_PATH}/xj3_aarch64/ successfully."
  fi

  # XJ3_X86_GCC485_PATH
  if [ `grep -c "${XJ3_X86_GCC485_FIND}" ${FIND_FILE}` == '0' ];then
    echo -e "export XJ3_X86_GCC485_PATH=${HOME}/.horizon/ddk/xj3_x86_64_gcc_4.8.5/" >> ${HOME}/.bashrc
    echo "Set XJ3_X86_GCC485_PATH environment variable value to ${DDK_LIB_PATH}/xj3_x86_64_gcc_4.8.5/ successfully."
  else
    sed -i 's#XJ3_X86_GCC485_PATH=.*#XJ3_X86_GCC485_PATH=${HOME}/.horizon/ddk/xj3_x86_64_gcc_4.8.5/#g' ${HOME}/.bashrc
    echo "Set XJ3_X86_GCC485_PATH environment variable value to ${DDK_LIB_PATH}/xj3_x86_64_gcc_4.8.5/ successfully."
  fi
  source ${HOME}/.bashrc

}

main() {
  confirm=y
  if [ -d "${HOME}/.horizon" ];then
    if [ $# -ge 1 ];then
      if [ "${1}" = "-h" -a "${1}" = "--help" ];then
        usage
      elif [ "${1}" = "-y" ];then
        get_sys_version
        set_env
        install_ddk_vcs
        install_host_package
        echo -e "${GREEN}Use [ddk_vcs list] to see more${PLAIN}"
      else
        usage
      fi
    else
      echo "${HOME}/.horizon is exist"
      echo -e "Do you want to reinstall all files? [${GREEN}Y${PLAIN}/${RED}n${PLAIN}]"
      read confirm
      if [[ ${confirm} =~ ^(y|Y|)$ ]];then
        get_sys_version
        set_env
        install_ddk_vcs
        install_host_package
        echo -e "${GREEN}Use [ddk_vcs list] to see more${PLAIN}"
      elif [ ${confirm} == "n" -o ${confirm} == "N" ];then
        echo -e "${RED}The file in the host_package directory are not installed in the ${HOME}/.horizon/${PLAIN}"
        exit 1
      fi
    fi
  else
    get_sys_version
    set_env
    install_ddk_vcs
    install_host_package
  fi

}
if [ "$BASH_VERSION" = "" ]; then
    echo "please change sh to bash"
    exit 1
else
    echo "sh is set to bash, ready to go"
fi
main "$@"
bash
