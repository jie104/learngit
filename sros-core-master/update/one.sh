#!/bin/bash
# file one.sh
# author pengjiali
# date 19-11-5.
# copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# describe 打包
# $1 == "release" 时会打包Matrix
# $1 == "test" 单元测试

#if ! lsb_release -r | grep '16.04' >/dev/null; then
#  echo -e "\033[31m build 本脚本只支持Ubuntu-16.04的系统!！！ \033[0m"
#  exit 1
#fi

echo "one.sh $1 $2"

GIT_VERSION=$(git --version | awk '{print $3}')
if dpkg --compare-versions "${GIT_VERSION}" lt "2.14.0"; then
  echo "当前git版本过老，当前版本是${GIT_VERSION},必须大于2.14.0 git submodule才好使，立马升级..."
  sudo add-apt-repository ppa:git-core/ppa -y
  sudo apt-get update
  sudo apt-get install git -y
  git config submodule.recurse true
fi

# 若子仓库不存在，同步下子仓库
if (ls -1qA ../cfg/sros-resources | grep -q .); then
  :
else
  git submodule update --init --recursive ../cfg/sros-resources
  git config submodule.recurse true
fi

if ! ./get_resources.sh generate; then
  echo -e "\033[31m 生成资源文件失败! \033[0m"
  exit 1
fi

# 清理content目录
rm -rf content/*

# 编译整包（nxp/tk1）
if [[ $2 == 'all' ]]; then

  # 编译TK1并保存到content
  ./sros_build.sh $1 'tk1'
  if [ $? -ne 0 ]; then
    exit $?
  fi

  ./generate_update.sh $1 'tk1' $2
  if [ $? -ne 0 ]; then
    exit $?
  fi

  # 编译NXP并保存到content
  ./sros_build.sh $1 'nxp'
  if [ $? -ne 0 ]; then
    exit $?
  fi

  ./generate_update.sh $1 'nxp' $2
  if [ $? -ne 0 ]; then
    exit $?
  fi

elif [[ $2 == 'nxp' ]]; then

  # 编译NXP并保存到content
  ./sros_build.sh $1 'nxp'
  if [ $? -ne 0 ]; then
    exit $?
  fi

  ./generate_update.sh $1 'nxp'
  if [ $? -ne 0 ]; then
    exit $?
  fi

else
  
  # 编译TK1并保存到content
  ./sros_build.sh $1 'tk1'
  if [ $? -ne 0 ]; then
    exit $?
  fi

  ./generate_update.sh $1 'tk1'
  if [ $? -ne 0 ]; then
    exit $?
  fi
  
fi

# 编译matrix并保存到content
./matrix_build.sh $1 $2
if [ $? -ne 0 ]; then
  exit $?
fi

# version.sh由cmake根据version.sh.in生成
source ../version.sh

# 自动读取当前版本git commit id
cur_git_commit_id=`git rev-parse HEAD`
short_id=${cur_git_commit_id:0:7}

# 程序打包加密成update包
cd content/
if [[ $2 == 'all' ]]; then
  update_file_name="SROS-${sros_version_str}-${short_id}.update"
else
  if [[ $2 == 'nxp' ]]; then
    update_file_name="SROS-${sros_version_str}-${short_id}-nxp.update"
  else
    update_file_name="SROS-${sros_version_str}-${short_id}-tk1.update"
  fi
fi

# 打包&加密
tar czf - * | openssl des3 -md md5 -salt -k SROS2016-update > ../${update_file_name}

# 清理现场
cd ..
if [[ -e sros_${short_id} ]]; then
  rm -rf sros_${short_id}
fi

if [[ -e sros_${short_id}-nxp ]]; then
  rm -rf sros_${short_id}-nxp
fi

echo "${update_file_name}"
