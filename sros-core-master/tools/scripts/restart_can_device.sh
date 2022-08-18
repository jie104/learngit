#!/usr/bin/env bash
# $1 == "3.10" 时需要加载驱动，默认为4.14，驱动已经装好了，只需要初始化can就新,若4.14的系统加载驱动会导致系统起不来

reload_can_driver() {
  rmmod apalis_tk1_k20_can gpio_apalis_tk1_k20 apalis_tk1_k20_adc apalis_tk1_k20_ts apalis_tk1_k20
#  modprobe apalis_tk1_k20

  # TODO(pengjjiali): vc400 考虑兼容问题
  insmod /lib/modules/3.10.40-2.8.4+gba29b06/kernel/drivers/mfd/apalis-tk1-k20.ko
  insmod /lib/modules/3.10.40-2.8.4+gba29b06/kernel/drivers/gpio/gpio-apalis-tk1-k20.ko
  insmod /lib/modules/3.10.40-2.8.4+gba29b06/kernel/drivers/net/can/apalis-tk1-k20-can.ko
  echo "# modprobe apalis_tk1_k20 ($?)"
  sleep 1
}

init_can_device() {
  ip link set can0 down
  echo "# ip link set can0 down ($?)"
  ip link set can0 type can bitrate 500000
  echo "# ip link set can0 type can bitrate 500000 ($?)"
  ip link set can0 up
  echo "# ip link set can0 up ($?)"
}

if [[ $1 == "3.10" ]]; then
  echo "reload can drivers..."

  reload_can_driver
fi

echo "init can device..."

init_can_device
