#!/usr/bin/env bash
# @File: ${NAME}.sh
# @Author: pengjiali
# @Date: 20-3-25
# @Copyright: Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
# @Describe: 启动本地sros，用于调试，可以将此脚本放到CLion启动sros需要启动的程序
# @Note: 注意默认本地sros编译后的位置在build-native目录下
#
# $1 == stop : 停止Matrix

# 停止Matrix
if [[ $1 == "stop" ]]; then
  pid=$(ps -ef | grep 'ui-server/code.py' | grep 'python2' | awk '{print $2}')
  if [[ $pid != "" ]];then kill $pid; fi
  pid=$(ps -ef | grep '/sros/web/websockify.py' | grep 'python' | awk '{print $2}')
  if [[ $pid != "" ]];then kill $pid; fi
  exit 0
fi

ROOT=''
#ROOT='/data/toradex/build/out-glibc/deploy/images/apalis-tk1/Apalis_TK1_LinuxConsoleImageV2.6/rootfs'

SROS_ROOT=${ROOT}'/sros'
HEAPPROFILE_DIR=${ROOT}'/sros/log/heapprofile'

cd ${SROS_ROOT}/bin

max_tk1_performance() {
  echo "max tk1 performance..."

  echo 0 >/sys/devices/system/cpu/cpuquiet/tegra_cpuquiet/enable

  echo 1 >/sys/devices/system/cpu/cpu0/online
  echo 1 >/sys/devices/system/cpu/cpu1/online
  echo 1 >/sys/devices/system/cpu/cpu2/online
  echo 1 >/sys/devices/system/cpu/cpu3/online

  echo performance >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
}

## fix boot up hangs on "a start job is running for Connection device"
#sleep 5

DB_FILE="/sros/db/main.db3"

function __readDB() {
  DB=$1
  ITEM=$2
  result=$(sqlite3 $DB "select value from config where key='$ITEM'")
  echo $result
}

function __get_meta_data() {
  KEY=$1
  r=$(sqlite3 ${DB_FILE} "SELECT value FROM meta WHERE key = '${KEY}'")
  echo ${r}
}

function __set_meta_data() {
  KEY=$1
  VALUE=$2
  r=$(sqlite3 ${DB_FILE} "UPDATE meta SET value = '${VALUE}' WHERE key = '${KEY}'")
}

echo "Run 'pre_startup_run.sh'..."
pre_startup_run_sh=${SROS_ROOT}/bin/pre_startup_run.sh
${pre_startup_run_sh}

echo "SROS startup..."

echo "Check for update..."

check_update() {
  date_str=$(date '+%Y%m%d_%H:%M:%S')
  if [ "$(ls -A ${SROS_ROOT}/update/)" = "" ]; then
    echo "[update] No update."
    return 0
  fi

  update_file_name=$(ls ${SROS_ROOT}/update | grep "^sros-")
  update_file=${SROS_ROOT}/update/${update_file_name}
  #update_file_md5=`md5sum ${update_file}`

  echo "[update] Find update file: ${update_file_name}"

  # 解压升级包
  cat ${update_file} | openssl des3 -md md5 -d -k SROS2016-update | tar xzf - -C ${SROS_ROOT}/../

  enable_sros_native_debug=$(__readDB "${DB_FILE}" "debug.enable_sros_native_debug")
  if [[ enable_sros_native_debug == 'False' ]]; then # 本地调试此处由于权限问题会解压失败
    if [ $? -ne 0 ]; then
      echo "[update] Extract update file failed!"
      # 清理update/
      rm -r ${SROS_ROOT}/update
      mkdir -p ${SROS_ROOT}/update
      echo "${date_str} md5_none no_sros_file fail" >>${SROS_ROOT}/update.log
      return -1
    fi
  fi

  echo "[update] Sync change to disk..."
  sync
  echo "[update] Extract update file successed!"

  # 提取SROS程序
  sros_file_name=$(ls ${SROS_ROOT}/update | grep "^sros_")
  db_file_name=$(ls ${SROS_ROOT}/update | grep "^main")
  sros_file=${SROS_ROOT}/update/${sros_file_name}
  sros_file_bin=${SROS_ROOT}/bin/${sros_file_name}

  db_dir=${SROS_ROOT}/db/
  db_dst_file=${SROS_ROOT}/db/${db_file_name}
  db_file=${SROS_ROOT}/update/${db_file_name}

  # init database
  if [ ! -d ${db_dir} ]; then
    mkdir -p ${db_dir}
  fi

  if [ ! -f ${db_dst_file} ]; then
    echo "[update] copying database file!"
    cp ${db_file} ${db_dir}
  fi

  # 拷贝数据库升级脚本
  if [ ! -d ${db_dir}/upgrade/ ]; then
    mkdir -p ${db_dir}/upgrade/
  fi
  cp -rfv ${SROS_ROOT}/update/db/db_schema_upgrade_* ${db_dir}/upgrade/
  cp -rfv ${SROS_ROOT}/update/db/db_upgrade_config_table.py ${db_dir}/

  cp -v ${sros_file} ${sros_file_bin}

  # 检查md5sum是否正确
  sros_file_md5=$(
    cd ${SROS_ROOT}/update
    md5sum ${sros_file_name}
  )
  md5sum_txt=$(cat ${SROS_ROOT}/update/md5sum_sros.txt)

  if [ ! "${sros_file_md5}" = "${md5sum_txt}" ]; then
    echo "[update] SROS checksum failed!"
    #    rm -r ${SROS_ROOT}/update
    #    mkdir -p ${SROS_ROOT}/update
    echo "${date_str} ${md5sum_txt} fail" >>${SROS_ROOT}/update.log
    return -2
  fi

  # 更新sros链接
  rm ${SROS_ROOT}/bin/sros
  ln -s ${sros_file_bin} ${SROS_ROOT}/bin/sros

  echo "[update] SROS update successed!"

  # 更新数据库结构

  # 从数据库中读取当前版本号
  cur_db_version=$(__get_meta_data "db.version")
  if [ -z "${cur_db_version}" ]; then
    echo "[update] no db version value, set version to 0"
    cur_db_version=0
  fi

  # 遍历db/upgrade/文件加，找到upgrade_sql的最新版本号
  upgrade_db_version=0
  for file in "${db_dir}upgrade"/*; do
    echo "[update] find db upgrade sql file : ${file}"
    name=$(basename "${file}" ".sql")
    version=$(cut -d'_' -f4 <<<"${name}")

    if [[ ${version} -gt ${upgrade_db_version} ]]; then
      upgrade_db_version=${version}
    fi
  done

  echo "[update] cur_db_version: ${cur_db_version}, upgrade_db_version: ${upgrade_db_version}"

  # 存在新版本的upgrade_sql，执行升级流程
  if [[ ${upgrade_db_version} -gt ${cur_db_version} ]]; then
    # 逐个版本升级，直到所有升级sql全部执行完
    v=$((${cur_db_version}))
    while [[ ${v} -lt ${upgrade_db_version} ]]; do
      v=$((${v} + 1))

      upgrade_sql="${db_dir}/upgrade/db_schema_upgrade_${v}.sql"
      echo "[update] execute upgrade sql: ${upgrade_sql}"

      sqlite3 ${db_dir}/main.db3 <<EOF
.read ${upgrade_sql}
EOF

    done

    # 更新版本号、升级日期等信息
    r=$(__set_meta_data "db.version" "${upgrade_db_version}")
    r=$(__set_meta_data "db.schema_upgrade_date" "$(date)")
  fi

  # 更新配置表
  db_config_csv_file=${SROS_ROOT}/update/config.csv
  if [ -f "$db_config_csv_file" ]; then
    echo "[update] Update db.config use config.csv..."

    # 由于config表中的id字段设置了unique属性，所以import不会覆盖已存在的记录
    # (由于该方法只能处理插入，不能处理更新、删除所以被废弃, 直接由下面的脚本来处理）
    #    sqlite3 ${SROS_ROOT}/db/main.db3 << EOF
    #.separator ","
    #.import ${db_config_csv_file} config
    #EOF
    python ${db_dir}db_upgrade_config_table.py ${db_dir}main.db3 ${db_config_csv_file}

    cp ${db_config_csv_file} ${SROS_ROOT}/db/db_config.csv
    echo "[update] Update db.config done"
  fi

  # 更新/sros/driver
  if [ -d ${SROS_ROOT}/update/driver ]; then
    echo "[update] Update driver dir"
    cp -rfv ${SROS_ROOT}/update/driver ${SROS_ROOT}/
  fi

  # 更新/sros/tools
  if [ -d ${SROS_ROOT}/update/tool ]; then
    echo "[update] Update tool dir"
    cp -rfv ${SROS_ROOT}/update/tool ${SROS_ROOT}/
  fi

  # 更新/sros/web
  if [ -d ${SROS_ROOT}/update/web ]; then
    echo "[update] Update web dir"
    cp -rfv ${SROS_ROOT}/update/web ${SROS_ROOT}/
  fi

  # 更新rootfs
  #  for f in `ls ${SROS_ROOT}/update/rootfs/`
  #  do
  #    echo "[update] f -> ${f}"
  #    if [ -d ${SROS_ROOT}/update/rootfs/${f} ]; then
  #      echo "[update] Executing ${SROS_ROOT}/update/rootfs/${f}/update_rootfs.sh"
  #      chmod +x ${SROS_ROOT}/update/rootfs/${f}/update_rootfs.sh
  #      ${SROS_ROOT}/update/rootfs/${f}/update_rootfs.sh
  #    fi
  #  done

  # 更新/sros/test
  if [ -d ${SROS_ROOT}/update/test ]; then
    echo "[update] Update test dir"
    cp -rfv ${SROS_ROOT}/update/test ${SROS_ROOT}/
  fi

  update_command_file=${SROS_ROOT}/update/update_command.sh
  if [ -f "$update_command_file" ]; then
    echo "[update] Found update_command.sh"
    echo "[update] Executeing update_command.sh..."
    chmod +x ${update_command_file}
    ${update_command_file}
    echo "[update] Finish update_command.sh"
  fi

  # 删除升级包
  echo "[update] remove update file ${update_file}"
  rm ${update_file}

  # 清理update/
  echo "[update] remove update files"
  rm -r ${SROS_ROOT}/update
  mkdir -p ${SROS_ROOT}/update

  # 记录升级信息
  echo "${date_str} ${md5sum_txt} success" >>${SROS_ROOT}/update.log

  return 1
}

check_timezone() {
  # set time zone to CST if it is not CST
  if [ -z $(ls /etc/localtime -l | awk -F' ' '{print $11}' | grep Hong_Kong) ]; then
    rm /etc/localtime
    ln -s /usr/share/zoneinfo/Asia/Hong_Kong /etc/localtime
  fi
}

prevent_time_rollback() {
  # 防止时间回流

  sros_last_save_timestamp=$(__get_meta_data "sys.time") # 从数据库中读取sros上次保存的时间戳
  cur_timestamp=$(date +%s)                              # 获取当前时间戳

  if [[ ${sros_last_save_timestamp} -gt ${cur_timestamp} ]]; then
    timedatectl set-time "$(date -d @${sros_last_save_timestamp} '+%F %T')"
    echo "[prevent time rollback] set date: $(date --date="@${sros_last_save_timestamp}")"
  fi
}

check_ntp() {
  # 设置NTP服务器
  echo "Check ntp"
  enable_ntp_sync=$(__readDB "${DB_FILE}" "time.enable_ntp_sync")
  ntp_server=$(__readDB "${DB_FILE}" "time.ntp_server")
  fallback_ntp_server=$(__readDB "${DB_FILE}" "time.fallback_ntp_server")

  if [[ $enable_ntp_sync == 'True' ]]; then
    echo "ntpdate ${ntp_server}"
    ntpdate ${ntp_server}

    # ntp服务器的配置默认被注释掉了，需要打开
    sed -i "s/^#NTP=/NTP=" /etc/systemd/timesyncd.conf
    sed -i "s/^#FallbackNTP=/FallbackNTP=" /etc/systemd/timesyncd.conf

    # 设置NTP服务器
    sed -i "/^NTP=/s/=.*/=${ntp_server}/" /etc/systemd/timesyncd.conf
    # 设置备用NTP服务器：ntp.ubuntu.com（91.189.94.4）、1.arch.pool.ntp.org（162.159.200.123）、ntp1.aliyun.com（120.25.115.20）
    # 直接设置域名，tk1无法解析，需要设置ip
    sed -i "/^FallbackNTP=/s/=.*/=91.189.94.4 162.159.200.123 120.25.115.20 ${fallback_ntp_server}/" /etc/systemd/timesyncd.conf
    timedatectl set-ntp true
  else
    timedatectl set-ntp false
  fi
}

clean_heapprofile_file() { # 删除heapprofile 生成的文件
  echo "Cleaning heap profile file before ${1} days..."

  dir=$(find ${HEAPPROFILE_DIR}/ -mtime +${1})

  if [[ -n "${dir}" ]]; then
    echo "files be removed:"
    echo "${dir}"

    $(find ${HEAPPROFILE_DIR}/ -mtime +${1} -exec rm -rf {} \;)
  fi
}

backup_heapprofile_file() { # 备份heapprofile文件
  echo "backup heapprofile file..."

  for file in ${HEAPPROFILE_DIR}/*; do
    if [[ -d ${file} ]]; then
      tar -cvzf ${file}.tar.gz -C ${file} .
      rm -vr ${file}
    fi
  done
}

clean_log_file() { # 根据当前系统时间来删除前面几条日志
  echo "Cleaning log file before ${1} days..."

  files=$(find ${SROS_ROOT}/log/ -mtime +${1} -name "*.log.*")

  if [ -n "${files}" ]; then
    echo "files be removed:"
    echo "${files}"

    $(find ${SROS_ROOT}/log/ -mtime +${1} -name "*.log.*" -exec rm -rf {} \;)
  fi

  # 保留10个*.json 文件，一个文件大约是1天的日志
  files2=$(ls ${SROS_ROOT}/log/*.json | grep -v "$(ls -u ${SROS_ROOT}/log/*.json | head -n 10)")

  if [ -n "${files2}" ]; then
    echo "log json files be removed:"
    echo "${files2}"

    $(rm ${files2})
  fi
}

force_clean_log_dir() { # 强制清除日志文件夹，当系统日期不正确的时候clean_log_file函数将会失效，此时需要此函数来强制清除
  echo "Force clean log dir...(${1})"

  files=$(ls -lt ${SROS_ROOT}/log/sros.* | tail +${1})

  if [ -n "${files}" ]; then
    echo "files be removed:"
    echo "${files}"

    $(ls -lt ${SROS_ROOT}/log/sros.* | tail +${1} | awk '{print "rm " $9}' | sh)
  fi
}

clean_bin_file() {
  echo "Cleaning old bin file...(${1})"

  files=$(ls -lt ${SROS_ROOT}/bin/sros_* | tail +${1})

  # 不删除sros软链接所指向的bin文件，因为若时间不正确的时候，sros指向的bin文件可能不是最新的
  if [ -L ${SROS_ROOT}/bin/sros ]; then
    sros_links_file=$(ls -l ${SROS_ROOT}/bin/sros | awk '{print $NF}')
    files=$(echo "$files" | grep -v $sros_links_file)
  fi

  if [ -n "${files}" ]; then
    echo "files be removed:"
    echo "${files}"

    echo "${files}" | awk '{print "rm " $9}' | sh
  fi
}

clean_core_file() {
  echo "Cleaning old core file...(${1})"

  files=$(ls -lt ${SROS_ROOT}/log/core-sros-* | tail +${1})

  if [ -n "${files}" ]; then
    echo "old coredump files be removed:"
    echo "${files}"

    $(ls -lt ${SROS_ROOT}/log/core-sros-* | tail +${1} | awk '{print "rm " $9}' | sh)
  fi

  # 其他的程序又有可能生产coredump，我们需要将其删除, 找了一圈，没有找到从源头上不产生coredump的方法，为了使sros产生coredump只能让所有的程序都产生coredump
  files=$(ls ${SROS_ROOT}/log/core-* | grep -v core-sros-)
  if [ -n "${files}" ]; then
    echo "other coredump files be removed:"
    echo "${files}"

    rm -v ${files}
  fi
}

recycle_diskspace() {
  echo "Recycle disk space..."

  # 清理掉1天前的所有日志
  clean_log_file 1

  clean_heapprofile_file 1

  # 强制清除log文件夹，当日期失效的时候，上面的函数也会失效
  force_clean_log_dir 12

  # 仅保留最近4-1=3次升级的bin文件
  clean_bin_file 4

  # 仅保留最近 1 次 core dump 的 core 文件
  clean_core_file 2
}

check_diskspace() {
  percent=$(df / | tail -1 | awk '{print $5}' | cut -d'%' -f1)

  if [ ${percent} -ge ${1} ]; then
    echo "[Warning] Disk space usage precent > ${1}%"

    recycle_diskspace
  fi
}

auto_install_drivers() {
  if [ ! -d ${SROS_ROOT}/driver ]; then
    echo "[driver] no driver dir"
    return -1
  fi

  # 逐个加载driver module
  for f in $(ls ${SROS_ROOT}/driver/); do
    echo "[driver] insmod ${SROS_ROOT}/driver/${f}"
    insmod ${SROS_ROOT}/driver/${f}
  done
}

# 清理日志文件（清理10天前的log）
clean_log_file 10

# 清理heapprofile文件
clean_heapprofile_file 10

# 检查磁盘空间占用（占用大于90%时回收磁盘空间）
check_diskspace 90

# 将heapprofile文件夹压缩备份
#backup_heapprofile_file

# 仅保留最近 5-1=4 次 core dump 的 core 文件
clean_core_file 5

# 检查是否有升级包，若有，进行升级操作
check_update

sleep 1

## kill the old script

# 指定core dump文件存放路径
#echo "${SROS_ROOT}/log/core-%e-%t" >/proc/sys/kernel/core_pattern

# TODO: 移植配置网卡ip到network-monitor中
# systemctl enable network-monitor.service

echo "Run 'pre_sros_run.sh'..."
pre_sros_run_sh=${SROS_ROOT}/bin/pre_sros_run.sh
${pre_sros_run_sh}

echo "Start websocket proxy..."
# 将5002端口的websocket数据转发到5001端口（SROS）
ret=$(ss -lntpd | grep 5002 | wc -l)
if [[ $ret -eq 1 ]]; then
  kill -9 $(ss -lntpd | grep 5002 | awk -F '[,=]' '{print $3}')
fi
${SROS_ROOT}/web/websockify.py 5002 127.0.0.1:5001 &

echo "Start ui-server..."
ret=$(ss -lntpd | grep 8080 | wc -l)
if [[ $ret -eq 1 ]]; then
  kill -9 $(ss -lntpd | grep 8080 | awk -F '[,=]' '{print $3}')
fi
cd ${SROS_ROOT}/web/ui-server # 需要进入此目录才能启动
python2 ${SROS_ROOT}/web/ui-server/code.py 8080 >/dev/null 2>&1 &

#echo "Start sros..."
#~/workspace/sros-core/build-native/sros --log_path=${SROS_ROOT}/log/
