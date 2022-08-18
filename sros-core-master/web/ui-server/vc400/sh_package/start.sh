#!/bin/bash

insmod /embedded/nxp/driver/asix.ko

sleep 2
#初始化
#ifconfig eth2 169.254.0.10
route add -net 169.254.0.0/16 dev eth2