#!/bin/bash

dataset_path=$1
run_type=$2
docker_name=""
version=v1.12.5

if [ -z "$dataset_path" ];then
  echo "Please specify the dataset path"
  exit
fi
dataset_path=$(readlink -f "$dataset_path")

if [ "$run_type" == "gpu" ];then
    start_method=nvidia-docker
    docker_name="ai_toolchain_ubuntu_gpu"
else
    start_method=docker
    docker_name="ai_toolchain_centos_7"
fi

docker_image=$(docker images |grep openexplorer/"${docker_name}" |grep "${version}" |awk -F '[ ]' '{print $ 1}')
if [ "$docker_image" == "" ];then
  docker pull openexplorer/${docker_name}:"${version}"
fi

echo "docker version is ${version}"
echo "dataset path is ${dataset_path}"

folder_name=$(basename  $(readlink -f "$(dirname "$0")"))
up_dir=$(readlink -f ../ )
cd "$up_dir" || exit

open_explorer_path=$(readlink -f "$folder_name" )
echo "open_explorer folder path is $open_explorer_path"

"${start_method}" run -it --rm \
  -v "$open_explorer_path":/open_explorer \
  -v "$dataset_path":/data/horizon_x3/data \
  openexplorer/${docker_name}:"${version}"
