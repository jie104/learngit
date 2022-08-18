#!/bin/bash

bash /sros/web/ui-server/vc400/sh_package/software_restart.sh
sleep 4

rm -r /lib/systemd/network/22-eth2.network
cp /sros/web/ui-server/vc400/22-eth2.network.ap /lib/systemd/network/
mv /lib/systemd/network/22-eth2.network.ap /lib/systemd/network/22-eth2.network

## 重启systemd-networkd，使配置生效
systemctl restart systemd-networkd
route add -net 169.254.0.0/16 dev eth2
echo "ap config successful!!!"