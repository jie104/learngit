#!/bin/bash

# $1 == "build" 时编译
# $1 == "test" 编译+测试
# $1 == "clear" 清除

if [[ $1 == "build" ]]; then
  if [ ! -d ./build ]; then
    mkdir ./build
  fi
  cd ./build && {
    # 请在此处指定打包前需要运行的单元测试
    if ! (
      cmake .. && make
    ); then
      echo -e "\033[31m 编译失败! \033[0m"
      exit 1
    fi
    cd ..
  }
  exit 0
fi


if [[ $1 == "test" ]]; then
  if [ ! -d ./build ]; then
    mkdir ./build
  fi
  cd ./build && {
    # 请在此处指定打包前需要运行的单元测试
    if ! (
      cmake .. && make  && ./unit_test
    ); then
      echo -e "\033[31m 编译失败! \033[0m"
      exit 1
    fi
    cd ..
  }
  exit 0
fi



if [[ $1 == "clear" ]]; then
  if [ ! -d ./build ]; then
    exit 0
  fi
  rm -r ./build
fi