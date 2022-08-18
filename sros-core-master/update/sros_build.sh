#!/bin/bash
# file one.sh
# author pengjiali
# date 19-11-5.
# copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# describe 打包
# $1 == "release"
# $2 == "tk1"
# $2 == "nxp"


# 屏蔽单元测试
#if [[ $1 == "test" ]]; then
#  if [ ! -d ../build-native ]; then
#    mkdir ../build-native
#  fi
#  cd ../build-native/ && {
#    # 请在此处指定打包前需要运行的单元测试
#    if ! (
#      cmake .. &&
#        make navigation_on_net_test -j7 && ctest -R navigation_on_net_test &&
#        make path_checker_test -j7 && ctest -R path_checker_test
#    ); then
#      echo -e "\033[31m 单元测试失败! \033[0m"
#      exit 1
#    fi
#    cd ..
#  }
#  exit 0
#fi

if [[ $2 == "nxp" ]]; then
  
  echo "[build]-- nxp build"
  # 执行cmake
  if [[ ! -d ../build-toradex_nxp ]]; then
    mkdir ../build-toradex_nxp
  fi
  cd ../build-toradex_nxp/ && pwd

  # 设置编译器
  cmake  -DCMAKE_TOOLCHAIN_FILE=../cmake/toradex-toolchain-aarch64.cmake -DCMAKE_BUILD_TYPE=Release  ..
  cmake  -DCMAKE_TOOLCHAIN_FILE=../cmake/toradex-toolchain-aarch64.cmake -DCMAKE_BUILD_TYPE=Release  ..
  cmake  -DCMAKE_TOOLCHAIN_FILE=../cmake/toradex-toolchain-aarch64.cmake -DCMAKE_BUILD_TYPE=Release  ..

else
  
  echo "[build]-- tk1 build"
  # 执行cmake
  if [[ ! -d ../build-toradex ]]; then
    mkdir ../build-toradex
  fi
  cd ../build-toradex/ && pwd
  
  # 设置编译器
  cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/toradex-toolchain.cmake -DCMAKE_BUILD_TYPE=Release ..
  cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/toradex-toolchain.cmake -DCMAKE_BUILD_TYPE=Release ..
  cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/toradex-toolchain.cmake -DCMAKE_BUILD_TYPE=Release ..
fi

function build_target() {
  TARGET=$1
  echo "Building ${TARGET}..."

  if ! make "${TARGET}" -j7; then
    echo -e "\033[31m Build target ${TARGET} failed! \033[0m"
    exit 1
  fi
}

build_target sros
if [[ $1 == 'release' ]]; then
  build_target sros_export_png_map
  build_target sros_map_json_to_map
  build_target sros_create_map_json
  build_target sros_monitor_file_to_json
  build_target srtos_log
  build_target src_test
  build_target sros_mvs_grap_image
fi
