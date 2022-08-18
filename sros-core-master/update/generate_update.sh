#!/usr/bin/env bash

# -------------------------------------------------
# Description: 打包生成'*.update升级包'
# $1: release
# $2: tk1
# $2: nxp
# $3: all 打整包
# -------------------------------------------------

echo "generate_update $(pwd)"
echo "./generate_update.sh $1 $2 $3"

# version.sh由cmake根据version.sh.in生成
source ../version.sh

# 自动读取当前版本git commit id
cur_git_commit_id=`git rev-parse HEAD`
short_id=${cur_git_commit_id:0:7}
str_cpu_version=$2

if [ ! -e 'content/sros/update' ]; then
    mkdir -p content/sros/update
fi

# 拷贝iap固件目录到update目录
if [ -d 'iap_bin' ]; then
  echo "IAP firmawre dist dir: $(pwd)/iap_bin"
  cp -r iap_bin content/sros/update/
fi

ls -l

# 拷贝tool相关文件
if [[ $1 == 'release' ]]; then
    if [[ ${str_cpu_version} == "nxp" ]]; then
        tool_dir=""
        if [[ $3 == "all" ]]; then
            tool_dir='content/sros/update/tool_64'
        else
            tool_dir='content/sros/update/tool'
        fi

        echo ${tool_dir}
        if [ ! -d ${tool_dir} ]; then
            mkdir -p ${tool_dir}
        fi
        cp -v ../build-toradex_nxp/bin_tools/* content/sros/update/tool_64
        cp -rv ../tools/scripts content/sros/update/tool_64
        cp -rv ../tools/frp content/sros/update/tool_64

    else

        if [ ! -d 'content/sros/update/tool' ]; then
            mkdir -p content/sros/update/tool
        fi
        cp -v ../build-toradex/bin_tools/* content/sros/update/tool
        cp -rv ../tools/scripts content/sros/update/tool
        cp -rv ../tools/frp content/sros/update/tool

    fi
fi

# 拷贝20180207到rootfs目录
if [ ! -e 'content/sros/update/rootfs' ]; then
    mkdir -p content/sros/update/rootfs
fi
cp -rL 20180207 content/sros/update/rootfs

# 打包lib库到不同目录
if [[ $3 != "all" ]]; then
    if [[ ${str_cpu_version} == "nxp" ]]; then
        cp -rL content/sros/update/rootfs/20180207/usr/lib/lib_64/* content/sros/update/rootfs/20180207/usr/lib/
        rm -rf content/sros/update/rootfs/20180207/usr/lib/lib_32
        rm -rf content/sros/update/rootfs/20180207/usr/lib/lib_64
    else
        cp -rL content/sros/update/rootfs/20180207/usr/lib/lib_32/* content/sros/update/rootfs/20180207/usr/lib/
        rm -rf content/sros/update/rootfs/20180207/usr/lib/lib_32
        rm -rf content/sros/update/rootfs/20180207/usr/lib/lib_64
    fi
fi

# 拷贝prepare_update.sh到/sros/update目录
cp -v prepare_update.sh content/sros/update/

# hack for libgflags.so missing
mkdir -p content/usr/lib

# 拷贝配置文件
cd ../cfg
./generate_db.sh
if [[ $? -ne 0 ]]; then
    echo '生成数据库升级脚本失败！'
    exit 1
fi
cd -

# 通过config.csv升级数据库（仅能升级原数据库不存在的记录）
cat ../cfg/config.csv | sed '/^$/d' > content/sros/update/config.csv

if [ ! -e 'content/sros/update/db' ]; then
    mkdir -p content/sros/update/db
fi

cp -v ../cfg/db_schema_upgrade_* content/sros/update/db/
cp -v ../cfg/db_schema_readonly_data.sql content/sros/update/db/
cp -v ../cfg/db_upgrade_config_table.py content/sros/update/db/

if [ ! -e 'content/sros/update/cfg/resources' ]; then
    mkdir -p content/sros/update/cfg/resources
fi
cp -vr ../cfg/resources content/sros/update/cfg/

cp -v ../rootfs/meta-sros/recipes-sros/files/startup.sh content/sros/update/
cp -v ../rootfs/meta-sros/recipes-sros/files/start_dump.sh content/sros/update/
cp -v ../rootfs/meta-sros/recipes-sros/files/stop_dump.sh content/sros/update/
cp -v ../rootfs/meta-sros/recipes-sros/files/network-monitor.sh content/sros/update/

# 拷贝驱动模块
mkdir -p content/sros/update/driver

# 拷贝sros程序
if [[ ${str_cpu_version} == "nxp" ]]; then
    cp ../build-toradex_nxp/sros ./sros_${short_id}-nxp
    cp -v ./sros_${short_id}-nxp content/sros/update/
else
    cp ../build-toradex/sros ./sros_${short_id}
    cp -v ./sros_${short_id} content/sros/update/
fi

# 拷贝测试程序
#if [ ! -e 'content/sros/update/test/util' ]; then
#    mkdir -p content/sros/update/test/util
#fi
#cp -v ../build-toradex/test/util/*_test content/sros/update/test/util
#cp -vr ../test/util/resources content/sros/update/test/util    # 将测试程序要用到的资源拷贝过去，如升级测试需要的升级包

# 计算md5校验
cd content/sros/update/
if [[ ${str_cpu_version} == "nxp" ]]; then
    md5sum sros_${short_id}-nxp > md5sum_sros-nxp.txt
else
    md5sum sros_${short_id} > md5sum_sros.txt
fi

