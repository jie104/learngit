# tk1 死机记录

### 日志1
```shell
Apr 24 15:35:40 apalis-tk1 startup.sh[26835]: Run 'pre_startup_run.sh'...
Apr 24 15:35:40 apalis-tk1 startup.sh[26835]: pre_startup_run.sh: Nothing.
Apr 24 15:35:40 apalis-tk1 startup.sh[26835]: SROS startup...
Apr 24 15:35:40 apalis-tk1 startup.sh[26835]: Check for update...
Apr 24 15:35:40 apalis-tk1 startup.sh[26835]: net.ipv4.tcp_retries2 = 6
Apr 24 15:35:40 apalis-tk1 startup.sh[26835]: Cleaning log file before 10 days...
Apr 24 15:35:40 apalis-tk1 startup.sh[26835]: Cleaning old core file...(5)
Apr 24 15:35:40 apalis-tk1 startup.sh[26835]: [update] No update.
Apr 24 15:35:41 apalis-tk1 startup.sh[26835]: [driver] insmod /sros/driver/ch341.ko
Apr 24 15:35:41 apalis-tk1 startup.sh[26835]: insmod: ERROR: could not insert module /sros/driver/ch341.ko: File exists
Apr 24 15:35:41 apalis-tk1 startup.sh[26835]: [driver] insmod /sros/driver/ftdi_sio.ko
Apr 24 15:35:41 apalis-tk1 startup.sh[26835]: insmod: ERROR: could not insert module /sros/driver/ftdi_sio.ko: File exists
Apr 24 15:35:41 apalis-tk1 startup.sh[26835]: reload can drivers...
Apr 24 15:35:41 apalis-tk1 systemd-networkd[26635]: can0: Lost carrier
Apr 24 15:35:41 apalis-tk1 startup.sh[26835]: rmmod: ERROR: Module apalis_tk1_k20_adc is not currently loaded
Apr 24 15:35:41 apalis-tk1 startup.sh[26835]: rmmod: ERROR: Module apalis_tk1_k20_ts is not currently loaded
Apr 24 15:35:41 apalis-tk1 kernel: gpio wake34 for gpio=82
Apr 24 15:35:41 apalis-tk1 kernel: apalis-tk1-k20 spi1.1: Apalis TK1 K20 MFD driver.
                                   Firmware version 0.11.
Apr 24 15:35:41 apalis-tk1 kernel: apalis_tk1_k20_adc: disagrees about version of symbol module_layout
Apr 24 15:35:41 apalis-tk1 kernel: apalis_tk1_k20: disagrees about version of symbol module_layout
Apr 24 15:35:41 apalis-tk1 kernel: apalis-tk1-k20-can apalis-tk1-k20-can.0: probed 0
Apr 24 15:35:41 apalis-tk1 kernel: apalis-tk1-k20-can apalis-tk1-k20-can.1: probed 1
Apr 24 15:35:41 apalis-tk1 kernel: gpiochip_add: registered GPIOs 856 to 1015 on device: generic
Apr 24 15:35:42 apalis-tk1 startup.sh[26835]: init can device...
Apr 24 15:35:42 apalis-tk1 kernel: apalis-tk1-k20 spi1.1: K20 FIFO Bug!
Apr 24 15:35:42 apalis-tk1 systemd-networkd[26635]: can0: Gained carrier
Apr 24 15:35:43 apalis-tk1 startup.sh[26835]: Configure network...
Apr 24 15:35:43 apalis-tk1 systemd[1]: Stopping Network Service...
Apr 24 15:35:43 apalis-tk1 systemd[1]: Stopped Network Service.
Apr 24 15:35:43 apalis-tk1 systemd[1]: Starting Network Service...
Apr 24 15:35:43 apalis-tk1 systemd-networkd[26939]: [/lib/systemd/network/20-eth0.network:7] Route is invalid, ignoring assignment: 0
Apr 24 15:35:43 apalis-tk1 systemd-networkd[26939]: eth0: Gained IPv6LL
Apr 24 15:35:43 apalis-tk1 systemd-networkd[26939]: enp1s0: Gained IPv6LL
Apr 24 15:35:43 apalis-tk1 systemd-networkd[26939]: Enumeration completed
Apr 24 15:35:43 apalis-tk1 systemd[1]: Started Network Service.
Apr 24 15:35:43 apalis-tk1 startup.sh[26835]: Run 'pre_sros_run.sh'...
Apr 24 15:35:43 apalis-tk1 avahi-daemon[316]: Withdrawing address record for 192.168.1.112 on eth0.
Apr 24 15:35:43 apalis-tk1 avahi-daemon[316]: Leaving mDNS multicast group on interface eth0.IPv4 with address 192.168.1.112.
Apr 24 15:35:43 apalis-tk1 startup.sh[26835]: pre_sros_run.sh: Nothing.
Apr 24 15:35:43 apalis-tk1 startup.sh[26835]: Sync change to disk...
Apr 24 15:35:43 apalis-tk1 avahi-daemon[316]: Interface eth0.IPv4 no longer relevant for mDNS.
-- Reboot --
```


---

### 日志2
```shell
Apr 25 18:44:43 apalis-tk1 systemd[1]: [/lib/systemd/system/sros.service:3] Failed to add dependency on multi-user.target,network.target, ignoring: Invalid argument
Apr 25 18:44:43 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nvfb.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 25 18:44:43 apalis-tk1 startup.sh[9494]: Created symlink /etc/systemd/system/multi-user.target.wants/udp_server.service → /lib/systemd/system/udp_server.service.
Apr 25 18:44:43 apalis-tk1 systemd[1]: Reloading.
Apr 25 18:44:43 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nv.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 25 18:44:43 apalis-tk1 systemd[1]: [/lib/systemd/system/sros.service:3] Failed to add dependency on multi-user.target,network.target, ignoring: Invalid argument
Apr 25 18:44:43 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nvfb.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 25 18:44:43 apalis-tk1 systemd[1]: Started SROS UDP Server Service.
Apr 25 18:44:43 apalis-tk1 systemd[1]: Stopping SROS UI Server Service...
Apr 25 18:44:43 apalis-tk1 systemd[1]: Stopped SROS UI Server Service.
Apr 25 18:44:44 apalis-tk1 startup.sh[9494]: Removed /etc/systemd/system/multi-user.target.wants/ui_server.service.
Apr 25 18:44:44 apalis-tk1 systemd[1]: Reloading.
Apr 25 18:44:44 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nv.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 25 18:44:44 apalis-tk1 systemd[1]: [/lib/systemd/system/sros.service:3] Failed to add dependency on multi-user.target,network.target, ignoring: Invalid argument
Apr 25 18:44:44 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nvfb.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 25 18:44:44 apalis-tk1 startup.sh[9494]: Created symlink /etc/systemd/system/multi-user.target.wants/ui_server.service → /lib/systemd/system/ui_server.service.
Apr 25 18:44:44 apalis-tk1 systemd[1]: Reloading.
Apr 25 18:44:44 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nv.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 25 18:44:44 apalis-tk1 systemd[1]: [/lib/systemd/system/sros.service:3] Failed to add dependency on multi-user.target,network.target, ignoring: Invalid argument
Apr 25 18:44:44 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nvfb.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 25 18:44:44 apalis-tk1 systemd[1]: Started SROS UI Server Service.
Apr 25 18:44:44 apalis-tk1 startup.sh[9494]: [update_rootfs] done
Apr 25 18:44:44 apalis-tk1 startup.sh[9494]: [driver] insmod /sros/driver/ch341.ko
Apr 25 18:44:44 apalis-tk1 startup.sh[9494]: insmod: ERROR: could not insert module /sros/driver/ch341.ko: File exists
Apr 25 18:44:44 apalis-tk1 startup.sh[9494]: [driver] insmod /sros/driver/ftdi_sio.ko
Apr 25 18:44:44 apalis-tk1 startup.sh[9494]: insmod: ERROR: could not insert module /sros/driver/ftdi_sio.ko: File exists
Apr 25 18:44:44 apalis-tk1 startup.sh[9494]: reload can drivers...
Apr 25 18:44:44 apalis-tk1 systemd-networkd[5382]: can0: Lost carrier
Apr 25 18:44:44 apalis-tk1 startup.sh[9494]: rmmod: ERROR: Module apalis_tk1_k20_adc is not currently loaded
Apr 25 18:44:44 apalis-tk1 startup.sh[9494]: rmmod: ERROR: Module apalis_tk1_k20_ts is not currently loaded
Apr 25 18:44:44 apalis-tk1 kernel: gpio wake34 for gpio=82
Apr 25 18:44:44 apalis-tk1 kernel: apalis-tk1-k20 spi1.1: Apalis TK1 K20 MFD driver.
                                   Firmware version 0.11.
Apr 25 18:44:44 apalis-tk1 kernel: apalis_tk1_k20: disagrees about version of symbol module_layout
Apr 25 18:44:44 apalis-tk1 kernel: gpiochip_add: registered GPIOs 856 to 1015 on device: generic
Apr 25 18:44:44 apalis-tk1 kernel: apalis_tk1_k20_adc: disagrees about version of symbol module_layout
Apr 25 18:44:44 apalis-tk1 kernel: apalis-tk1-k20-can apalis-tk1-k20-can.0: probed 0
Apr 25 18:44:44 apalis-tk1 kernel: apalis-tk1-k20-can apalis-tk1-k20-can.1: probed 1
Apr 25 18:44:45 apalis-tk1 startup.sh[9494]: init can device...
Apr 25 18:44:45 apalis-tk1 kernel: apalis-tk1-k20 spi1.1: K20 FIFO Bug!
Apr 25 18:44:45 apalis-tk1 systemd-networkd[5382]: can0: Gained carrier
Apr 25 18:44:46 apalis-tk1 startup.sh[9494]: Configure network...
Apr 25 18:44:46 apalis-tk1 systemd[1]: Stopping Network Service...
Apr 25 18:44:46 apalis-tk1 systemd[1]: Stopped Network Service.
Apr 25 18:44:46 apalis-tk1 systemd[1]: Starting Network Service...
Apr 25 18:44:46 apalis-tk1 systemd-networkd[9765]: [/lib/systemd/network/20-eth0.network:7] Route is invalid, ignoring assignment: 0
Apr 25 18:44:46 apalis-tk1 systemd-networkd[9765]: eth0: Gained IPv6LL
Apr 25 18:44:46 apalis-tk1 systemd-networkd[9765]: enp1s0: Gained IPv6LL
Apr 25 18:44:46 apalis-tk1 systemd-networkd[9765]: Enumeration completed
Apr 25 18:44:46 apalis-tk1 systemd[1]: Started Network Service.
Apr 25 18:44:46 apalis-tk1 startup.sh[9494]: Run 'pre_sros_run.sh'...
Apr 25 18:44:46 apalis-tk1 avahi-daemon[244]: Withdrawing address record for 192.168.1.112 on eth0.
Apr 25 18:44:46 apalis-tk1 avahi-daemon[244]: Leaving mDNS multicast group on interface eth0.IPv4 with address 192.168.1.112.
Apr 25 18:44:46 apalis-tk1 avahi-daemon[244]: Interface eth0.IPv4 no longer relevant for mDNS.
Apr 25 18:44:46 apalis-tk1 startup.sh[9494]: pre_sros_run.sh: Nothing.
Apr 25 18:44:46 apalis-tk1 startup.sh[9494]: Sync change to disk...
-- Reboot --

```



---


### log3
```shell
Apr 29 11:36:46 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nvfb.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 29 11:36:46 apalis-tk1 systemd[1]: Reloading.
Apr 29 11:36:46 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nv.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 29 11:36:46 apalis-tk1 systemd[1]: [/lib/systemd/system/sros.service:3] Failed to add dependency on multi-user.target,network.target, ignoring: Invalid argument
Apr 29 11:36:46 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nvfb.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 29 11:36:46 apalis-tk1 systemd[1]: Stopping SROS UDP Server Service...
Apr 29 11:36:46 apalis-tk1 python[13570]: {'vehicle_serial_no': '8BVSR12420029', 'hw_version': 'NA', 'serial_no': '31746f241a4c0212', 'fw_version': 'NA', 'vehicle_action_unit': 'riser', 'mac_address': '00:14
Apr 29 11:36:46 apalis-tk1 systemd[1]: Stopped SROS UDP Server Service.
Apr 29 11:36:46 apalis-tk1 systemd[1]: Reloading.
Apr 29 11:36:46 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nv.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 29 11:36:46 apalis-tk1 systemd[1]: [/lib/systemd/system/sros.service:3] Failed to add dependency on multi-user.target,network.target, ignoring: Invalid argument
Apr 29 11:36:46 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nvfb.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 29 11:36:46 apalis-tk1 systemd[1]: Reloading.
Apr 29 11:36:46 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nv.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 29 11:36:46 apalis-tk1 systemd[1]: [/lib/systemd/system/sros.service:3] Failed to add dependency on multi-user.target,network.target, ignoring: Invalid argument
Apr 29 11:36:46 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nvfb.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 29 11:36:46 apalis-tk1 systemd[1]: Started SROS UDP Server Service.
Apr 29 11:36:46 apalis-tk1 systemd[1]: Stopping SROS UI Server Service...
Apr 29 11:36:46 apalis-tk1 systemd[1]: Stopped SROS UI Server Service.
Apr 29 11:36:46 apalis-tk1 systemd[1]: Reloading.
Apr 29 11:36:46 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nv.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 29 11:36:46 apalis-tk1 systemd[1]: [/lib/systemd/system/sros.service:3] Failed to add dependency on multi-user.target,network.target, ignoring: Invalid argument
Apr 29 11:36:46 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nvfb.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 29 11:36:47 apalis-tk1 systemd[1]: Reloading.
Apr 29 11:36:47 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nv.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 29 11:36:47 apalis-tk1 systemd[1]: [/lib/systemd/system/sros.service:3] Failed to add dependency on multi-user.target,network.target, ignoring: Invalid argument
Apr 29 11:36:47 apalis-tk1 systemd[1]: Configuration file /lib/systemd/system/nvfb.service is marked executable. Please remove executable permission bits. Proceeding anyway.
Apr 29 11:36:47 apalis-tk1 systemd[1]: Started SROS UI Server Service.
Apr 29 11:36:47 apalis-tk1 systemd-networkd[13698]: can0: Lost carrier
Apr 29 11:36:47 apalis-tk1 kernel: gpio wake34 for gpio=82
Apr 29 11:36:47 apalis-tk1 kernel: apalis-tk1-k20 spi1.1: Apalis TK1 K20 MFD driver.
                                   Firmware version 0.11.
Apr 29 11:36:47 apalis-tk1 kernel: apalis_tk1_k20_adc: disagrees about version of symbol module_layout
Apr 29 11:36:47 apalis-tk1 kernel: apalis_tk1_k20: disagrees about version of symbol module_layout
Apr 29 11:36:47 apalis-tk1 kernel: apalis-tk1-k20-can apalis-tk1-k20-can.0: probed 0
Apr 29 11:36:47 apalis-tk1 kernel: apalis-tk1-k20-can apalis-tk1-k20-can.1: probed 1
Apr 29 11:36:47 apalis-tk1 kernel: gpiochip_add: registered GPIOs 856 to 1015 on device: generic
Apr 29 11:36:48 apalis-tk1 kernel: apalis-tk1-k20 spi1.1: K20 FIFO Bug!
Apr 29 11:36:48 apalis-tk1 systemd-networkd[13698]: can0: Gained carrier
Apr 29 11:36:49 apalis-tk1 systemd[1]: Stopping Network Service...
Apr 29 11:36:49 apalis-tk1 systemd[1]: Stopped Network Service.
Apr 29 11:36:49 apalis-tk1 systemd[1]: Starting Network Service...
Apr 29 11:36:49 apalis-tk1 systemd-networkd[17365]: [/lib/systemd/network/20-eth0.network:7] Route is invalid, ignoring assignment: 0
Apr 29 11:36:49 apalis-tk1 systemd-networkd[17365]: eth0: Gained IPv6LL
Apr 29 11:36:49 apalis-tk1 systemd-networkd[17365]: enp1s0: Gained IPv6LL
Apr 29 11:36:49 apalis-tk1 systemd-networkd[17365]: Enumeration completed
Apr 29 11:36:49 apalis-tk1 systemd[1]: Started Network Service.
Apr 29 11:36:49 apalis-tk1 avahi-daemon[233]: Withdrawing address record for 192.168.1.112 on eth0.
Apr 29 11:36:49 apalis-tk1 avahi-daemon[233]: Leaving mDNS multicast group on interface eth0.IPv4 with address 192.168.1.112.
Apr 29 11:36:49 apalis-tk1 avahi-daemon[233]: Interface eth0.IPv4 no longer relevant for mDNS.
Apr 29 11:36:49 apalis-tk1 avahi-daemon[233]: Joining mDNS multicast group on interface eth0.IPv4 with address 192.168.1.112.
Apr 29 11:36:49 apalis-tk1 avahi-daemon[233]: New relevant interface eth0.IPv4 for mDNS.
Apr 29 11:36:49 apalis-tk1 avahi-daemon[233]: Registering new address record for 192.168.1.112 on eth0.IPv4.
Apr 29 11:36:49 apalis-tk1 avahi-daemon[233]: Withdrawing address record for 192.168.11.1 on rndis0.
Apr 29 11:36:49 apalis-tk1 avahi-daemon[233]: Leaving mDNS multicast group on interface rndis0.IPv4 with address 192.168.11.1.
Apr 29 11:36:49 apalis-tk1 avahi-daemon[233]: Interface rndis0.IPv4 no longer relevant for mDNS.
-- Reboot --
Apr 29 11:38:14 apalis-tk1 systemd-journald[162]: Runtime journal (/run/log/journal/) is 8.0M, max 96.3M, 88.3M free.
Apr 29 11:38:14 apalis-tk1 kernel: Booting Linux on physical CPU 0x0
Apr 29 11:38:14 apalis-tk1 kernel: Initializing cgroup subsys cpu
```

### tk1无响应
现象：
- sros连续运行了4天，一切正常，重启sros后，一直无法登录sros。
- 查看sros日志，并没有新日志生成
- systemctl restart ui_server，没有反应，用网页登录页面也进不了
- top
```shell
top - 09:50:37 up 6 days, 23:27,  1 user,  load average: 45.51, 42.71, 32.36
Tasks: 130 total,   9 running, 120 sleeping,   0 stopped,   1 zombie
%Cpu(s):  0.0 us,  1.8 sy,  0.0 ni, 98.2 id,  0.0 wa,  0.0 hi,  0.0 si,  0.0 st
KiB Mem :  1970424 total,   323388 free,   758040 used,   888996 buff/cache
KiB Swap:        0 total,        0 free,        0 used.  1166031 avail Mem 

  PID USER      PR  NI    VIRT    RES    SHR S  %CPU %MEM     TIME+ COMMAND                                                                                                  
21938 root      20   0 1194564 678044  11824 D   0.0 34.4  10407:00 sros                                                                                                     
  170 root      20   0   45328  20456  20104 R 100.0  1.0 143:26.03 systemd-journal                                                                                          
21936 root      20   0   10960   7808   2880 S   0.0  0.4   0:32.51 python                                                                                                   
  517 root      20   0   10900   7536   3316 S   0.0  0.4   1:52.18 python                                                                                                   
  520 root      20   0   36980   6448   3028 S   0.0  0.3   1:43.06 python                                                                                                   
    1 root      20   0   23348   3264   2244 D   0.0  0.2   0:10.87 systemd                                                                                                  
  380 root      20   0    8004   2488   1908 D   0.0  0.1   0:10.88 connmand                                                                                                 
  386 root      20   0    5264   2208   1712 S   0.0  0.1   0:00.27 ofonod                                                                                                   
21932 systemd+  20   0    5080   2184   1808 S   0.0  0.1   0:00.54 systemd-network                                                                                          
24655 root      20   0    4408   2012   1676 S   0.0  0.1   0:01.14 sshd                                                                                                     
24657 root      20   0    4836   1832   1368 S   0.0  0.1   0:00.07 sh                                                                                                       
  381 root      20   0    4276   1748   1476 S   0.0  0.1   0:00.66 systemd-logind                                                                                           
  385 avahi     20   0    4032   1680   1284 S   0.0  0.1   0:11.98 avahi-daemon                                                                                             
  185 root      20   0   12496   1636   1024 S   0.0  0.1   0:01.22 systemd-udevd                                                                                            
  537 root      20   0    8180   1560   1176 S   0.0  0.1   0:02.11 wpa_supplicant                                                                                           
  337 message+  20   0    3776   1536   1132 S   0.0  0.1   0:00.27 dbus-daemon                                                                                              
29674 root      20   0    4920   1480    924 R  10.5  0.1   0:00.79 top                                                                                                      
21807 root      20   0    3156   1384   1068 S   0.0  0.1   0:00.03 startup.sh                                                                                               
  466 rpcuser   20   0    2692   1356    824 S   0.0  0.1   0:00.02 rpc.statd 
```
- kill -s 9 170没有响应
- reboot 也没有响应
- journalctl -f 
```shell
root@apalis-tk1:/sros/log# journalctl -f
-- Logs begin at Mon 2019-11-11 07:17:38 HKT. --
Nov 11 09:29:03 apalis-tk1 startup.sh[21807]:     @   0x43cdd0 (unknown)
Nov 11 09:29:14 apalis-tk1 startup.sh[21807]: 192.168.71.1 - - [11/Nov/2019 09:29:14] 192.168.71.1: Plain non-SSL (ws://) WebSocket connection
Nov 11 09:29:14 apalis-tk1 startup.sh[21807]: 192.168.71.1 - - [11/Nov/2019 09:29:14] connecting to: 127.0.0.1:5001
Nov 11 09:29:16 apalis-tk1 startup.sh[21807]: 192.168.71.1 - - [11/Nov/2019 09:29:16] 192.168.71.1: Plain non-SSL (ws://) WebSocket connection
Nov 11 09:29:16 apalis-tk1 startup.sh[21807]: 192.168.71.1 - - [11/Nov/2019 09:29:16] connecting to: 127.0.0.1:5001
Nov 11 09:29:16 apalis-tk1 startup.sh[21807]: 192.168.71.1 - - [11/Nov/2019 09:29:16] 192.168.71.1: Plain non-SSL (ws://) WebSocket connection
Nov 11 09:29:16 apalis-tk1 startup.sh[21807]: 192.168.71.1 - - [11/Nov/2019 09:29:16] connecting to: 127.0.0.1:5001
Nov 11 09:29:19 apalis-tk1 startup.sh[21807]: 192.168.71.1 - - [11/Nov/2019 09:29:19] 192.168.71.1: Plain non-SSL (ws://) WebSocket connection
Nov 11 09:29:19 apalis-tk1 startup.sh[21807]: 192.168.71.1 - - [11/Nov/2019 09:29:19] connecting to: 127.0.0.1:5001
Nov 11 09:29:31 apalis-tk1 kernel: ------------[ cut here ]------------

```
- journalctl -n 100 也没响应

断电重启解决问题