#!/usr/bin/env bash

# 相对于startup.sh
cur_dir=/sros/update/rootfs/20180207
pymodbus_dir=/usr/lib/python3.5/site-packages/pymodbus

echo "[update_rootfs] copy file..."

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

echo "current cpu : ${strCpu}"
if [[ ${strCpu} == ${strTk1} ]]; then
  # 删除存在的pymodbus
  if [ -d ${pymodbus_dir} ]; then
    echo "[update_rootfs] remove old pymodbus"
    rm -rf ${pymodbus_dir}
  fi
fi

echo "[update_rootfs] sync to disk..."
sync

if [[ ${strCpu} == ${strTk1} ]]; then
  if [[ -d ${cur_dir}/usr/lib/lib_32 ]]; then
    cp -drvf ${cur_dir}/usr/lib/lib_32/* /usr/lib/
  else
    cp -drvf ${cur_dir}/usr/lib /usr/
  fi
else
  if [[ -d ${cur_dir}/usr/lib/lib_64 ]]; then
    cp -drvf ${cur_dir}/usr/lib/lib_64/* /usr/lib/
  else
    cp -drvf ${cur_dir}/usr/lib /usr/
  fi
fi
cp -drvf ${cur_dir}/lib /
cp -drvf ${cur_dir}/etc /

echo "[update_rootfs] sync to disk..."
sync

echo "[update_rootfs] reload service..."

systemctl daemon-reload

systemctl stop auto_http_server.service
systemctl disable auth_http_server.service
systemctl enable auth_http_server.service
systemctl start auth_http_server.service

systemctl stop udp_server.service
systemctl disable udp_server.service
systemctl enable udp_server.service
systemctl start udp_server.service

systemctl stop ui_server.service
systemctl disable ui_server.service
systemctl enable ui_server.service
systemctl start ui_server.service

systemctl stop frpc.service
systemctl disable frpc.service
# systemctl enable frpc.service
# systemctl start frpc.service

systemctl stop mqtt.service
systemctl disable mqtt.service
systemctl enable mqtt.service
systemctl start mqtt.service

echo "[update_rootfs] done"
