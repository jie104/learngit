#!/bin/bash

#需要知道gpio恢复出厂设置需要多久生效

#rmmod /home/root/asix.ko
#sleep 1
#insmod /home/root/asix.ko
gpioset 3 26=0
sleep 8
gpioset 3 26=1
#初始化
ifconfig eth2 169.254.0.10
route add -net 169.254.0.0/16 dev eth2
sleep 30
#使用脚本启动无线网卡ap
python /sros/web/ui-server/vc400/ap_conf_script.py

bash /sros/web/ui-server/vc400/sh_package/software_restart.sh
sleep 4

rm -r /lib/systemd/network/22-eth2.network
cp /sros/web/ui-server/vc400/22-eth2.network.ap /lib/systemd/network/
mv /lib/systemd/network/22-eth2.network.ap /lib/systemd/network/22-eth2.network

## 重启systemd-networkd，使配置生效
systemctl restart systemd-networkd
route add -net 169.254.0.0/16 dev eth2
echo "ap config successful!!!"