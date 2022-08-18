#!/bin/bash

ROOT=''
#ROOT='/data/toradex/build/out-glibc/deploy/images/apalis-tk1/Apalis_TK1_LinuxConsoleImageV2.6/rootfs'

SROS_ROOT=${ROOT}'/sros'
HEAPPROFILE_DIR=${ROOT}'/sros/log/heapprofile'

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

kernel_release=`uname -r`
kernel_release_trunk=${kernel_release:0:4} # 现阶段有两个版本一个是4.14一个是3.10
chip_version=$(getChipVersion)

echo "kernel : $kernel_release,chip_version : $chip_version"

max_tk1_performance() {
  echo "max tk1 performance..."

  echo 0 > /sys/devices/system/cpu/cpuquiet/tegra_cpuquiet/enable

  echo 1 > /sys/devices/system/cpu/cpu0/online
  echo 1 > /sys/devices/system/cpu/cpu1/online
  echo 1 > /sys/devices/system/cpu/cpu2/online
  echo 1 > /sys/devices/system/cpu/cpu3/online

  echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
}

## fix boot up hangs on "a start job is running for Connection device"
#sleep 5

DB_FILE="/sros/db/main.db3"

function __readDB(){
    DB=$1;ITEM=$2
    result=`sqlite3 $DB "select value from config where key='$ITEM'"`
    echo $result
}

function __get_meta_data() {
    KEY=$1
    r=`sqlite3 ${DB_FILE} "SELECT value FROM meta WHERE key = '${KEY}'"`
    echo ${r}
}

function __set_meta_data() {
    KEY=$1;VALUE=$2
    r=`sqlite3 ${DB_FILE} "UPDATE meta SET value = '${VALUE}' WHERE key = '${KEY}'"`
}

# 生成/更新proc/info文件
function update_info_file() {
  PROC_ROOT="${SROS_ROOT}/proc"
  mkdir -p ${PROC_ROOT}

  vehicle_file="${PROC_ROOT}/info/serial/vehicle"
  mkdir -p "${PROC_ROOT}/info/serial"
  touch ${vehicle_file}
  vehicle_serial=$(__readDB "${DB_FILE}"  "main.vehicle_serial_no")
  echo -n ${vehicle_serial} > ${vehicle_file}

  nickname_file="${PROC_ROOT}/info/nickname"
  touch ${nickname_file}
  nickname=$(__readDB "${DB_FILE}"  "main.nickname")
  echo -n ${nickname} > ${nickname_file}
}

echo "Run 'pre_startup_run.sh'..."
pre_startup_run_sh=${SROS_ROOT}/bin/pre_startup_run.sh
${pre_startup_run_sh}

echo "SROS startup..."

echo "Check for update..."

check_update() {
  date_str=`date '+%Y%m%d_%H:%M:%S'`
  if [ "`ls -A ${SROS_ROOT}/update/`" = "" ]; then
    echo "[update] No update."
    return 0
  fi

  # check the .update_record_iap flag
  UPDATE_RECORD_FLAG=`hexdump  ${SROS_ROOT}/.update_record_iap | awk -F ' '  '{printf $2}' `
  if [ "${UPDATE_RECORD_FLAG}" -eq "0011" ]; then
    echo "[update] SROS not update at this step, IAP update, write back 0xF1"
    echo "[update] UPDATE_RECORD_FLAG=${UPDATE_RECORD_FLAG}"
    return 0
  fi

  if [ -f "${SROS_ROOT}/update/db_import/main.db3" ]; then # 只是替换数据库的情况
    cp -v ${SROS_ROOT}/update/db_import/main.db3 /sros/db/

    # 清理update/
    echo "[update] remove update files"
    rm -r ${SROS_ROOT}/update
    mkdir -p ${SROS_ROOT}/update
    return 1
  fi

  update_file_name=`ls ${SROS_ROOT}/update | grep "^sros-"`
  update_file=${SROS_ROOT}/update/${update_file_name}

  echo "[update] Find update file: ${update_file_name}"

  # 解压升级包
  cat ${update_file} | openssl des3 -md md5 -d -k SROS2016-update | tar xzf - -C ${SROS_ROOT}/../

  if [ $? -ne 0 ]; then
    echo "[update] Extract update file failed!"
    # 清理update/
    rm -r ${SROS_ROOT}/update
    mkdir -p ${SROS_ROOT}/update
    echo "${date_str} md5_none no_sros_file fail" >> ${SROS_ROOT}/update.log
    echo -e -n "\xFF" > ${SROS_ROOT}/update_record
    return 1
  fi

  # 清理非当前核心板库文件
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

  echo "[update] Sync change to disk..."
  sync
  echo "[update] Extract update file successed!"

  # 提取SROS程序
  sros_file_name=`ls ${SROS_ROOT}/update | grep "^sros_"`
  db_file_name=`ls ${SROS_ROOT}/update | grep "^main"`
  sros_file=${SROS_ROOT}/update/${sros_file_name}
  sros_file_bin=${SROS_ROOT}/bin/${sros_file_name}

  db_dir=${SROS_ROOT}/db
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
  cp -rfv ${SROS_ROOT}/update/db/db_schema_readonly_data.sql ${db_dir}/upgrade/
  cp -rfv ${SROS_ROOT}/update/db/db_upgrade_config_table.py ${db_dir}/

  cp -v ${sros_file} ${sros_file_bin}

  # 检查md5sum是否正确
  sros_file_md5=`cd ${SROS_ROOT}/update; md5sum ${sros_file_name}`
  if [[ ${chip_version} == 'nxp' ]]; then
    md5sum_txt=`cat ${SROS_ROOT}/update/md5sum_sros-nxp.txt`
  else
    md5sum_txt=`cat ${SROS_ROOT}/update/md5sum_sros.txt`
  fi

  if [ ! "${sros_file_md5}" = "${md5sum_txt}" ]; then
    echo "[update] SROS checksum failed!"
#    rm -r ${SROS_ROOT}/update
#    mkdir -p ${SROS_ROOT}/update
    echo "${date_str} ${md5sum_txt} fail" >> ${SROS_ROOT}/update.log
    echo -e -n "\xFF" > ${SROS_ROOT}/update_record
    return 2
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
  for file in "${db_dir}/upgrade"/*
  do
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
    v=$(( ${cur_db_version} ))
    while [[ ${v} -lt ${upgrade_db_version} ]]; do
      v=$(( ${v} + 1 ))

      upgrade_sql="${db_dir}/upgrade/db_schema_upgrade_${v}.sql"
      echo "[update] execute upgrade sql: ${upgrade_sql}"

      sqlite3 ${db_dir}/main.db3 << EOF
.read ${upgrade_sql}
EOF

    done

    # 更新版本号、升级日期等信息
    r=$(__set_meta_data "db.version" "${upgrade_db_version}")
    r=$(__set_meta_data "db.schema_upgrade_date" "$(date)")
  fi

  readonly_data_upgrade_file=${db_dir}/upgrade/db_schema_readonly_data.sql
  sqlite3 ${db_dir}/main.db3 << EOF
.read ${readonly_data_upgrade_file}
EOF

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

    python ${db_dir}/db_upgrade_config_table.py ${db_dir}/main.db3 ${db_config_csv_file}
    if [ $? -ne 0 ]; then
      echo "[update] Update db.config done"
    else
      echo "[update] Update db.config failed!!!"
    fi
    rm ${db_dir}/argparse.py # 将零时的库删掉 TODO: tk1升级到sros_v4.6.0+以后： remove me!!!
    rm ${db_dir}/argparse.pyc # 将零时的库删掉 TODO: tk1升级到sros_v4.6.0+以后： remove me!!!

    cp ${db_config_csv_file} ${SROS_ROOT}/db/db_config.csv
  fi

  # 更新/sros/driver
  if [ -d ${SROS_ROOT}/update/driver ]; then
    echo "[update] Update driver dir"
    cp -rfv ${SROS_ROOT}/update/driver ${SROS_ROOT}/
  fi

  # 更新/sros/tools
  if [ -d ${SROS_ROOT}/update/tool ]; then
    echo "[update] Update tool dir"
    if [[ ${chip_version} == 'nxp' ]]; then
      cp -rfv ${SROS_ROOT}/update/tool_64/* ${SROS_ROOT}/tool/
    else
      cp -rfv ${SROS_ROOT}/update/tool ${SROS_ROOT}/
    fi
  fi

  # 更新/sros/web
  if [ -d ${SROS_ROOT}/update/web ]; then
    echo "[update] Update web dir"
    cp -rfv ${SROS_ROOT}/update/web ${SROS_ROOT}/
  fi

  # 更新rootfs
  for f in `ls ${SROS_ROOT}/update/rootfs/`
  do
    echo "[update] f -> ${f}"
    if [ -d ${SROS_ROOT}/update/rootfs/${f} ]; then
      echo "[update] Executing ${SROS_ROOT}/update/rootfs/${f}/update_rootfs.sh"
      chmod +x ${SROS_ROOT}/update/rootfs/${f}/update_rootfs.sh
      ${SROS_ROOT}/update/rootfs/${f}/update_rootfs.sh
    fi
  done

  # 更新/sros/cfg
  if [ -d ${SROS_ROOT}/update/cfg ]; then
    echo "[update] Update cfg dir"
    cp -rfv ${SROS_ROOT}/update/cfg/resources ${SROS_ROOT}/cfg
  fi

  # 更新/sros/test
  if [ -d ${SROS_ROOT}/update/test ]; then
    echo "[update] Update test dir"
    cp -rfv ${SROS_ROOT}/update/test ${SROS_ROOT}/
  fi

  # 更新dump启动脚本
  if [[ -e ${SROS_ROOT}/update/start_dump.sh ]] && [[ -e ${SROS_ROOT}/update/stop_dump.sh ]]; then
    cp -v ${SROS_ROOT}/update/start_dump.sh ${SROS_ROOT}/bin/
    cp -v ${SROS_ROOT}/update/stop_dump.sh ${SROS_ROOT}/bin/
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
  echo "${date_str} ${md5sum_txt} success" >> ${SROS_ROOT}/update.log
  echo -e -n "\xFF" > ${SROS_ROOT}/update_record
  return 1
}

check_timezone() {
    # set time zone to CST if it is not CST
    if [ -z `ls /etc/localtime -l | awk -F' ' '{print $11}'|grep Hong_Kong` ]; then
        rm /etc/localtime
        ln -s /usr/share/zoneinfo/Asia/Hong_Kong /etc/localtime
    fi
}

prevent_time_rollback() {
    # 防止时间回流

    sros_last_save_timestamp=$(__get_meta_data "sys.time") # 从数据库中读取sros上次保存的时间戳
    cur_timestamp=`date +%s` # 获取当前时间戳

    if [[ ${sros_last_save_timestamp} -gt ${cur_timestamp} ]]; then
        timedatectl set-time "`date -d @${sros_last_save_timestamp} '+%F %T'`"
        echo "[prevent time rollback] set date: `date --date="@${sros_last_save_timestamp}"`"
    fi
}

check_ntp() {
    # 设置NTP服务器
    echo "Check ntp"
    enable_ntp_sync=$(__readDB "${DB_FILE}"  "time.enable_ntp_sync")
    ntp_server=$(__readDB "${DB_FILE}"  "time.ntp_server")
    fallback_ntp_server=$(__readDB "${DB_FILE}"  "time.fallback_ntp_server")

    if [[ $enable_ntp_sync == 'True' ]]; then
        echo "ntpdate ${ntp_server}"
        ntpdate ${ntp_server}

        # ntp服务器的配置默认被注释掉了，需要打开
        sed -i "s/^#NTP=/NTP=/" /etc/systemd/timesyncd.conf
        sed -i "s/^#FallbackNTP=/FallbackNTP=/" /etc/systemd/timesyncd.conf

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

clean_other_process_log() { # 删除其他程序生产的日志
    echo "Cleaning other process log file before ${1} days..."

    for file in ${SROS_ROOT}/log/*; do
        if [[ -d ${file} ]];then
            dir=`find ${file}/ -mtime +${1}`

            if [[ -n "${dir}" ]];then
                echo "files be removed:"
                echo "${dir}"

                `find ${file}/ -mtime +${1} -exec rm -rf {} \;`
            fi
        fi
    done

    echo "Clean the Orbbec SDK log file /Log/*.log ..."
    rm /Log/*.log
}

backup_heapprofile_file() { # 备份heapprofile文件
    echo "backup heapprofile file..."

    for file in ${HEAPPROFILE_DIR}/*; do
        if [[ -d ${file} ]];then
            if [[ `cat ${file} | wc -l` -ne 0 ]]; then # 空文件夹就不要压缩了
                tar -cvzf ${file}.tar.gz -C ${file} .
                rm -vr ${file}
            else
                rm ${file} -r
            fi
        fi
    done
}

clean_log_file() { # 根据当前系统时间来删除前面几条日志
    echo "Cleaning log file before ${1} days..."

    files=`find ${SROS_ROOT}/log/ -mtime +${1} -name "*.log.*"`

    if [ -n "${files}" ];then
        echo "files be removed:"
        echo "${files}"

        `find ${SROS_ROOT}/log/ -mtime +${1} -name "*.log.*" -exec rm -rf {} \;`
    fi

    # 保留10个*.json 文件，一个文件大约是1天的日志
    files2=`ls ${SROS_ROOT}/log/*.json | grep -v "$(ls -u ${SROS_ROOT}/log/*.json | head -n 10)"`

    if [ -n "${files2}" ];then
        echo "log json files be removed:"
        echo "${files2}"

        `rm ${files2}`
    fi
}

force_clean_log_dir() { # 强制清除日志文件夹，当系统日期不正确的时候clean_log_file函数将会失效，此时需要此函数来强制清除
    echo "Force clean log dir...(${1})"

    files=`ls -lt ${SROS_ROOT}/log/sros.* | tail +${1}`

    if [ -n "${files}" ];then
        echo "files be removed:"
        echo "${files}"

        `ls -lt ${SROS_ROOT}/log/sros.* | tail +${1} | awk '{print "rm " $9}' | sh`
    fi
}

clean_bin_file() {
    echo "Cleaning old bin file...(${1})"

    files=`ls -lt ${SROS_ROOT}/bin/sros_* | tail +${1}`

    # 不删除sros软链接所指向的bin文件，因为若时间不正确的时候，sros指向的bin文件可能不是最新的
    if [ -L ${SROS_ROOT}/bin/sros ];then
        sros_links_file=`ls -l ${SROS_ROOT}/bin/sros | awk '{print $NF}'`
        files=`echo "$files" | grep -v $sros_links_file`
    fi

    if [ -n "${files}" ];then
        echo "files be removed:"
        echo "${files}"

        echo "${files}" | awk '{print "rm " $9}' | sh
    fi
}

clean_core_file() {
    echo "Cleaning old core file...(${1})"

    files=`ls -lt ${SROS_ROOT}/log/core-sros-* | tail +${1}`

    if [ -n "${files}" ];then
        echo "old coredump files be removed:"
        echo "${files}"

        `ls -lt ${SROS_ROOT}/log/core-sros-* | tail +${1} | awk '{print "rm " $9}' | sh`
    fi

    # 其他的程序又有可能生产coredump，我们需要将其删除, 找了一圈，没有找到从源头上不产生coredump的方法，为了使sros产生coredump只能让所有的程序都产生coredump
    files=`ls ${SROS_ROOT}/log/core-* | grep -v core-sros-`
    if [ -n "${files}" ];then
        echo "other coredump files be removed:"
        echo "${files}"

        rm -v ${files}
    fi
}

recycle_diskspace() {
    echo "Recycle disk space..."

    # 清理掉1天前的所有日志
    clean_log_file 1

    clean_other_process_log 1

    # 强制清除log文件夹，当日期失效的时候，上面的函数也会失效
    force_clean_log_dir 12

    # 仅保留最近4-1=3次升级的bin文件
    clean_bin_file 4

    # 仅保留最近 1 次 core dump 的 core 文件
    clean_core_file 2
}

check_diskspace() {
    percent=`df / | tail -1 | awk '{print $5}' | cut -d'%' -f1`

    if [ ${percent} -ge ${1} ];then
        echo "[Warning] Disk space usage precent > ${1}%"

        recycle_diskspace
    fi
}

auto_install_drivers() {
  if [ ! -d ${SROS_ROOT}/driver ]; then
    echo "[driver] no driver dir"
    return 1
  fi

  # 逐个加载driver module
  for f in `ls ${SROS_ROOT}/driver/`
  do
    echo "[driver] insmod ${SROS_ROOT}/driver/${f}"
    insmod ${SROS_ROOT}/driver/${f}
  done
}

# 生成proc/info文件
update_info_file

# 重新设置net.ipv4.tcp_retries2，减少tcp超时重试次数
sysctl -w net.ipv4.tcp_retries2=6

# 检查并设置时区
check_timezone

# 防止时间回流
prevent_time_rollback

# 检查并设置NTP
check_ntp

# 清理日志文件（清理10天前的log）
clean_log_file 10

# 清理除sros进程产生的日志
clean_other_process_log 10

# 检查磁盘空间占用（占用大于90%时回收磁盘空间）
check_diskspace 90

# 将heapprofile文件夹压缩备份
backup_heapprofile_file

# 仅保留最近 5-1=4 次 core dump 的 core 文件
clean_core_file 5

# 检查是否有升级包，若有，进行升级操作
check_update

# 自动加载/sros/driver/下的所有.ko
if [[ ${kernel_release_trunk} == 3.10 ]]; then
  auto_install_drivers #NOTE： 此处简单处理了，若4.14的系统需要加载驱动的话，应该将这些驱动放到一个子文件夹中，不同内核加载不同的驱动
fi

# 挂载ch340驱动
if [[ ${chip_version} == 'nxp' ]]; then
  insmod /usr/lib/ch341.ko
fi

sleep 1

restart_can_device_sh="/sros/tool/scripts/restart_can_device.sh"
if [ -f ${restart_can_device_sh} ]; then
  ${restart_can_device_sh} ${kernel_release_trunk}
fi

# 由于硬件设计缺陷，VC400要将GPIO 4脚拉高，不然解除不了急停，在VC300中GPIO4用于用户GPIO
vehicle_controller_type=$(__readDB "${DB_FILE}"  "main.vehicle_controller_type")
if [[ ${vehicle_controller_type} == "VC400" ]]; then
  echo "power on set GPIO 4 high!"
  echo 109 > /sys/class/gpio/export
  echo "out" > /sys/class/gpio/gpio109/direction
  echo 1 > /sys/class/gpio/gpio109/value
else
  # VC300要将GPIO 6脚拉低，mcu复位脚，要默认输出低，否则mcu不工作
  echo "power on set GPIO 6 LOW!"
  echo 354 > /sys/class/gpio/export
  echo "out" > /sys/class/gpio/gpio354/direction
  echo 0 > /sys/class/gpio/gpio354/value
fi

if [[ ${chip_version} == 'nxp' ]]; then
  # 解除急停
  gpioset 0 13=1

  #nxp can config
  ip link set can0 type can bitrate 500000 restart-ms 100
  ip link set can1 type can bitrate 500000 restart-ms 100
  sleep 1
  ifconfig can0 up
  ifconfig can1 up

  # 配置网络
  eth1_ip=$(__readDB "${DB_FILE}"  "network.enp3s0_ip")
  eth1_ip_type=$(__readDB "${DB_FILE}"  "network.enp3s0_ip_type")
  eth1_ip_gateway=$(__readDB "${DB_FILE}"  "network.enp3s0_gateway")
  eth1_netmask_prefix_len=$(__readDB "${DB_FILE}"  "network.enp3s0_netmask_prefixlen")

  eth1_DHCP="no"
  if [ "${eth1_ip_type}" = "DHCP" ]; then
    eth1_DHCP="yes"
  fi

  if [ -z "$eth1_ip" ]; then
    eth1_ip="192.168.1.112"
  fi

  if [ -z "$eth1_netmask_prefix_len" ]; then
    eth1_netmask_prefix_len="24"
  fi
fi

echo "Configure network..."

# 读取用户设置的ip地址
enp3s0_ip=$(__readDB "${DB_FILE}"  "network.enp3s0_ip")
enp3s0_ip_type=$(__readDB "${DB_FILE}"  "network.enp3s0_ip_type")
enp3s0_ip_gateway=$(__readDB "${DB_FILE}"  "network.enp3s0_gateway")
enp3s0_netmask_prefix_len=$(__readDB "${DB_FILE}"  "network.enp3s0_netmask_prefixlen")

eth0_ip=$(__readDB "${DB_FILE}"  "network.eth0_ip")
eth0_ip_type=$(__readDB "${DB_FILE}"  "network.eth0_ip_type")
eth0_ip_gateway=$(__readDB "${DB_FILE}"  "network.eth0_gateway")
eth0_netmask_prefix_len=$(__readDB "${DB_FILE}"  "network.eth0_netmask_prefixlen")

enp3s0_DHCP="no"
if [ "${enp3s0_ip_type}" = "DHCP" ]; then
  enp3s0_DHCP="yes"
fi

eth0_DHCP="no"
if [ "${eth0_ip_type}" = "DHCP" ]; then
  eth0_DHCP="yes"
fi

if [ -z "$eth0_ip" ]; then
  eth0_ip="192.168.2.113"
fi

if [ -z "$enp3s0_ip" ]; then
  enp3s0_ip="192.168.1.112"
fi

if [ -z "$enp3s0_netmask_prefix_len" ]; then
  enp3s0_netmask_prefix_len="24"
fi

function update_network_config() {
  DHCP=$1
  ip=$2
  gateway=$3
  netmask=$4
  file=$5

  if [ "${DHCP}" = "yes" ]; then
    sed -i "/^DHCP/s/=.*/=yes/" ${file}
    sed -i "/^Address/s/.*/#Address=/" ${file}
    sed -i "/^Gateway/s/.*/#Gateway=/" ${file}
  else
    sed -i "/^DHCP/s/=.*/=no/" ${file}
    sed -i "/Address/s/.*/Address=${ip}\/${netmask}/" ${file}
    sed -i "/Gateway/s/.*/Gateway=${gateway}/" ${file}
  fi

  if [ -z "${gateway}" ]; then
    sed -i "/^Gateway/s/.*/#Gateway=/" ${file}
  fi

}

# 重新配置网卡ip配置文件
enp3s0_network_file=${ROOT}"/lib/systemd/network/10-enp3s0.network"
enp1s0_network_file=${ROOT}"/lib/systemd/network/11-enp1s0.network"
eth0_network_file=${ROOT}"/lib/systemd/network/20-eth0.network"
eth1_network_file=${ROOT}"/lib/systemd/network/21-eth1.network"

# enp3s0 与 enp1s0 为同一块物理网卡，只是由于硬件原因（不同生产批次）导致名称发生变化
update_network_config "${enp3s0_DHCP}" "${enp3s0_ip}" "${enp3s0_ip_gateway}" "${enp3s0_netmask_prefix_len}" "${enp3s0_network_file}"
update_network_config "${enp3s0_DHCP}" "${enp3s0_ip}" "${enp3s0_ip_gateway}" "${enp3s0_netmask_prefix_len}" "${enp1s0_network_file}"

update_network_config "${eth0_DHCP}" "${eth0_ip}" "${eth0_ip_gateway}" "${eth0_netmask_prefix_len}" "${eth0_network_file}"
update_network_config "${eth1_DHCP}" "${eth1_ip}" "${eth1_ip_gateway}" "${eth1_netmask_prefix_len}" "${eth1_network_file}"
# 重启systemd-networkd，使配置生效
systemctl restart systemd-networkd

# 判断没有网关ip为192.168.71.1的默认路由，没有则添加一个默认路由
if [[ ${chip_version} == 'nxp' ]]; then
  /sros/tool/scripts/default_route_add.sh &
fi

## kill the old script

# 指定core dump文件存放路径
echo "${SROS_ROOT}/log/core-%e-%t" > /proc/sys/kernel/core_pattern

if [[ ${chip_version} == 'nxp' ]]; then
  ## 解决nxp  sros崩溃不生产core问题
  ulimit -c unlimited
  ulimit -S unlimited
  # nxp急停拉高
  gpioset 0 13=1
fi

# TODO: 移植配置网卡ip到network-monitor中
# systemctl enable network-monitor.service

echo "Run 'pre_sros_run.sh'..."
pre_sros_run_sh=${SROS_ROOT}/bin/pre_sros_run.sh
${pre_sros_run_sh}

echo "Sync change to disk..."
sync

echo "Start websocket proxy..."
# 将5002端口的websocket数据转发到5001端口（SROS）
${SROS_ROOT}/web/websockify.py 5002 127.0.0.1:5001 &

# 触摸屏
screen_v2_instance_count=`ps -ef | grep 'screen_v2.py' | wc -l`
if [[ ${screen_v2_instance_count} -eq 2 ]]; then
    kill -9 `ps -ef | grep 'screen_v2.py' | head -n 1 | awk '{print $2}'`
fi
enable_touch_screen=$(__readDB "${DB_FILE}"  "hmi.enable_touch_screen")
touch_screen_version=$(__readDB "${DB_FILE}"  "hmi.touch_screen_version")
enable_modbus_tcp=$(__readDB "${DB_FILE}"  "modbus.enable_modbus_tcp")
if [[ ${enable_touch_screen} == "True" ]] && [[ ${touch_screen_version} == "v2" ]] && [[ ${enable_modbus_tcp} == "True" ]]; then
    python3 /sros/tool/scripts/screen_v2.py &
fi

rm -rv /sros/lib # 默认直接用系统中的库，这个只是临时使用

# 设置环境变量(添加升级后新依赖的位置)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/sros/lib:/sros/update/rootfs/20180207/usr/lib
if [[ ${chip_version} == 'nxp' ]]; then
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/sros/update/rootfs/20180207/usr/lib/lib_64
else
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/sros/update/rootfs/20180207/usr/lib/lib_32
fi

# 关闭tk1休眠模式，全功率运行
if [[ ${kernel_release_trunk} == 3.10 ]]; then
  max_tk1_performance
fi

# 设置启用gperftools/heap-profiler
#new_heapprofile_dir=${HEAPPROFILE_DIR}/heapprofle-`date "+%Y%m%d-%H%M%S"`
#mkdir -pv ${new_heapprofile_dir}
#echo "heapprofile dir is ${new_heapprofile_dir}
#export HEAPPROFILE=${new_heapprofile_dir}/heap-sros
#export HEAP_PROFILE_ALLOCATION_INTERVAL=0 # 107374182400 大概12分钟生成一次heapperfile 文件，主要是这个参数
#export HEAPPROFILESIGNAL=12
# `killall -12 sros` to dump heap


# 设置启用gperftools/cpu-profiler
# 将根目录下的tcmalloc_minimal替换为tcmalloc_and_profiler,将cpu-profiler相关的代码包含进来
# 执行‘killall -12 sros’命令开始cpu-profiler采样，再次执行‘killall -12 sros’停止采样
# 打开下面三行代码，采样相关参数
#export CPUPROFILE=sros.prof # 生成的cpu-profiler文件
#export CPUPROFILESIGNAL=12 # 开始和结束生成cpu-profiler的信号
#export CPUPROFILE_FREQUENCY=100 # 采样频率，本值越大，生成的cpu-profiler越详细


# 添加检查maps数据脚本
sh /sros/web/ui-server/check_maps_file_and_db.sh

echo "SROS running..."

# nxp初始化脚本
if [[ ${chip_version} == 'nxp' ]]; then
  sh /sros/web/ui-server/vc400/sh_package/start.sh
fi

# 启动sros主程序
${SROS_ROOT}/bin/sros --log_path=${SROS_ROOT}/log/




