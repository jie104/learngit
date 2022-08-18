#!/bin/bash
# Author: pengjiali
# describe: 此脚本将会解压到/sros/update目录下，在升级前有sros调用，作用是做一些升级需要处理的事情

echo "enter $0"

# 获取核心板类型
function getChipVersion() {
  strCpu="";
  strTk1="tk1"
  strNxp="nxp"
  # 根据hostname区分不同芯片
  host_name=$(echo $(hostname) | grep ${strTk1})
  if [[ ${host_name} == "" ]]; then
    strCpu=${strNxp}
  else
    strCpu=${strTk1}
  fi
  echo $strCpu
}

# 清理非当前核心板库文件
chip_version=$(getChipVersion)
for file in $(ls /sros/update | grep -E '(^sros_|^md5sum_)'); do
  echo "file:$file"
  if [ ${chip_version} == 'tk1' ]; then
    # 删除nxp可执行程序
    file_name=`echo ${file} | grep "nxp"`
    if [[ $file_name != "" ]]; then
      rm -f "/sros/update/$file"
    fi
  elif [ ${chip_version} == 'nxp' ]; then
    # 删除nxp可执行程序
    file_name=`echo ${file} | grep "nxp"`
    if [[ $file_name == "" ]]; then
      rm -f "/sros/update/$file"
    fi
  fi
done

# 兼容旧版本库环境，拷贝对应核心板库文件到/usr/lib下
cur_dir=/sros/update/rootfs/20180207
if [ ${chip_version} == 'tk1' ]; then
  if [[ -d ${cur_dir}/usr/lib/lib_32 ]]; then
    cp -drvf ${cur_dir}/usr/lib/lib_32/* ${cur_dir}/usr/lib/
  fi
elif [ ${chip_version} == 'nxp' ]; then
  if [[ -d ${cur_dir}/usr/lib/lib_64 ]]; then
    cp -drvf ${cur_dir}/usr/lib/lib_64/* ${cur_dir}/usr/lib/
  fi
fi

# 删除原有废弃文件和日志
rm -rf /usr/lib/orbbec.ini
rm -rf /usr/lib/OniFile.ini
rm -rf /Log/*.log

echo "exit $0"
