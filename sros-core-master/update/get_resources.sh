#!/usr/bin/env bash
# @File: ${NAME}.sh
# @Author: pengjiali
# @Date: 20-4-9
# @Copyright: Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
# @Describe:
# $1 == "generate" 时会重新生成，否则只是拷贝

SROS_RESOURCES_DIR="../cfg/sros-resources"
SROS_PROTOBUF_DIR="../core/proto"

if [[ $1 == 'generate' ]]; then
    cd ${SROS_RESOURCES_DIR} && pwd
    echo -e "\033[32m 开始生成资源文件... \033[0m"
    ./generate_all.sh
    if [[ $? -ne 0 ]]; then
        echo -e "\033[31m 生成资源文件失败！ \033[0m"
        exit 1
    fi
    echo -e "\033[32m 生成资源文件成功！ \033[0m"
    cd -
else
    if    ls -1qA ../cfg/sros-resources | grep -q .
    then  :
    else
        echo -e "\033[31m 资源文件夹是空的，无法拷贝！ \033[0m"
        exit 1
    fi
fi

echo -e "\033[32m 开始拷贝资源... \033[0m"

cd ../ # 进入sros-core目录
SROS_CORE_DIR=$(pwd)
cd -
ln -sf ../../cfg/sros-resources/protobuf/main.proto "${SROS_CORE_DIR}"/core/proto/main.proto
ln -sf ../../cfg/sros-resources/protobuf/monitor.proto "${SROS_CORE_DIR}"/core/proto/monitor.proto

cp -v ${SROS_RESOURCES_DIR}/db/* ../cfg/ # 拷贝数据库升级脚本
cp -v ${SROS_RESOURCES_DIR}/target/db_schema_readonly_data.sql ../cfg/

ln -sf ../cfg/sros-resources/target/error_code.h "${SROS_CORE_DIR}"/core/error_code.h
ln -sf ../cfg/sros-resources/target/fault_code.h "${SROS_CORE_DIR}"/core/fault_code.h
ln -sf ../cfg/sros-resources/target/music_id.h "${SROS_CORE_DIR}"/core/music_id.h
ln -sf ../../cfg/sros-resources/target/device_id.h "${SROS_CORE_DIR}"/core/device/device_id.h
cp -v ${SROS_RESOURCES_DIR}/target/config.csv ../cfg/ # 拷贝配置文件

if [[ ! -d ../cfg/resources/ ]];then
    mkdir ../cfg/resources
fi

if [[ ! -d ../cfg/resources/action/ ]];then
    mkdir ../cfg/resources/action
fi

cp -v ${SROS_RESOURCES_DIR}/action/src/* ../cfg/resources/action
cp -v ${SROS_RESOURCES_DIR}/action/sros/* ../cfg/resources/action
cp -v ${SROS_RESOURCES_DIR}/action/eac/* ../cfg/resources/action

cp -vr ${SROS_RESOURCES_DIR}/device ../cfg/resources/
cp -r ${SROS_RESOURCES_DIR}/target/error_code.csv ../cfg/resources/
cp -r ${SROS_RESOURCES_DIR}/target/error_code.json ../cfg/resources/
cp -r ${SROS_RESOURCES_DIR}/target/fault_code.csv ../cfg/resources/
cp -r ${SROS_RESOURCES_DIR}/target/fault_code.json ../cfg/resources/
cp -r ${SROS_RESOURCES_DIR}/i18n/error_code_i18n.json ../cfg/resources/
cp -r ${SROS_RESOURCES_DIR}/i18n/fault_code_i18n.json ../cfg/resources/

echo -e "\033[32m 拷贝资源成功！ \033[0m"

cd ../cfg/sros-resources/bin && {
  pwd
  python3 check_error_code_scripts.py

  if [[ $? -ne 0 ]]; then
    echo '检查错误码不在国际化配置中失败！'
    exit 1
  fi

  cd ..
}