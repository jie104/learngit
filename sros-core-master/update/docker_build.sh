#!/bin/bash
# file docker_build.sh
# author pengjiali
# date 20-7-28.
# copyright Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
# describe 用docker打包sros
# $1 == "release" 打包Matrix

./check_docker_container.sh sros-docker
ret=$?
if [ $ret == 0 ]; then
  : # sros-native-debug容器已经启动
elif [ $ret == 2 ]; then # 容器已经被停止
  docker start sros-docker
elif [ $ret == 3 ]; then # 容器不存在
  docker run --name sros-docker --restart always --privileged=true --cap-add sys_ptrace -p 127.0.0.1:2222:22 -p 127.0.0.1:8000:80 -p 127.0.0.1:5002:5002 -p 127.0.0.1:5020:502 -v ~/workspace:/workspace -it 1.116.113.42:5000/sros/native:latest /start.sh debug &
fi

docker exec --user $(id -u):$(id -g) -it sros-docker /build.sh $1 $2
