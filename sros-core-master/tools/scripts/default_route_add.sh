#!/bin/bash


## 因为关闭控制器connman服务 所以路由表更改需要重新添加路由
# 判断没有网关ip为192.168.71.1的默认路由，没有则添加一个默认路由
vehicle_controller_type=$(head -n 1 /sys/bus/soc/devices/soc0/soc_id)
if [[ ${vehicle_controller_type} == "i.MX8QM" ]]; then
  count=0
  while true
  do
    # 查看路由表是否有默认路由192.168.71.1
    default_gw=`route -n | awk 'NR==3{print}' | awk '{if ($1=="0.0.0.0" && $2=="192.168.71.1") print $2}'`
    # 查看路由表是否出现192.168.71.0
    route1=`route -n | awk 'NR==3{print}' | awk '{if ($1=="192.168.71.0") print $2}'`
    route2=`route -n | awk 'NR==4{print}' | awk '{if ($1=="192.168.71.0") print $2}'`

    # 没有默认路由192.168.71.1的情况下，添加默认路由192.168.71.1
    if [ "${default_gw}" = "192.168.71.1" ]; then
      echo "default_route_add.sh: 默认路由192.168.71.1已添加"
    else
      if [[ "0.0.0.0" = "0.0.0.0" || "${route2}" = "0.0.0.0" ]] ;then
        echo "default_route_add.sh: 添加默认路由 route add default gw 192.168.71.1"
        route add default gw 192.168.71.1
      fi
    fi
    sleep 2
    count=`expr $count + 1`
    # 超过5分钟查看到默认路由192.168.71.1的情况下，跳出死循环结束脚本
    if [ "${default_gw}" = "192.168.71.1" ] && [ $count -gt 150 ]; then
      echo "default_route_add.sh: 默认路由192.168.71.1已存在"
      break
    fi
    echo "default_route_add.sh: 等待默认路由192.168.71.1稳定"
  done
fi
