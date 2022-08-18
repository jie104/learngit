#!/bin/bash

#使用脚本启动无线网卡wlan
bash /sros/web/ui-server/vc400/sh_package/software_restart.sh
sleep 4

rm -r /lib/systemd/network/22-eth2.network
cp /sros/web/ui-server/vc400/22-eth2.network.wlan /lib/systemd/network/
mv /lib/systemd/network/22-eth2.network.wlan /lib/systemd/network/22-eth2.network

## 重启systemd-networkd，使配置生效
systemctl restart systemd-networkd
route add -net 169.254.0.0/16 dev eth2
echo "wlan config successful!!!"
