# -*- coding:utf-8 -*-
# file config.py
# author YangHuaiDong
# date 2022/2/28 下午3:25
# copyright Copyright (c) 2021 Standard Robots Co., Ltd. All rights reserved.
# describe

# 数据库绝对目录
DB_PATH = "/sros/db/main.db3"
SROS_PATH = "/sros/web/ui-server/vc400"
# 在工程下使用相对地址
AP_CONF_PATH = "{}/xml_package/children/ap_conf.xml".format(SROS_PATH)
WLAN_CONF_PATH = "{}/xml_package/children/wlan_conf.xml".format(SROS_PATH)
BRIDGE_CONF_PATH = "{}/xml_package/children/change_bridge.xml".format(SROS_PATH)
DELETE_WLAN_PATH = "{}/xml_package/children/delete_wlan.xml".format(SROS_PATH)

CHILREN_CONF_PATH = "{}/xml_package/children/conf.xml".format(SROS_PATH)

CONF_PATH = "{}/xml_package/config.xml".format(SROS_PATH)
STATUS_PATH = "{}/xml_package/status.xml".format(SROS_PATH)

AP_STATE_PATH = "{}/xml_package/children/setting_ap.xml".format(SROS_PATH)

NETWORK_WLAN_PATH = "{}/22-eth2.network.wlan".format(SROS_PATH)
NETWORK_WLAN_DATA = {"DHCP": "no", "Address": "192.168.108.244/22", "Gateway": "192.168.110.1"}

WLAN_DATA = {  # WLAN参数配置
    "Basic": {
        "Network Name": "SR_office",  # 连接wlan的账号名称
        "WPAx Passphrase": "SR@office",  # 密码
    },
    "Interface_wlan": {
        "DHCP Client": "Disabled",
        "IP Address": "10.10.70.94/22",  # wlan静态ip
        "Default Gateway": "10.10.70.1",
    },
    "HTTP Server": {"Port": "8080"},
    "Interface_ap": {
        "State": "Disabled",
    },
}

AP_DATA = {  # AP参数配置
    "Access Point": {
        "SSID": "standard001",
        "Passphrase": "standard"
    },
    "Interface_ap": {
        "State": "Enabled",
        "IP Address": "192.168.0.1/24",
        "DHCP IP Address Range": {
            "Start": "192.168.0.6",
            "End": "192.168.0.220"
        },
    },
    "HTTP Server": {"Port": "8080"},
    "Interface_wlan": {
        "State": "Disabled",
    }
}

BRIDGE_DATA = {  # 桥接参数配置
    "ap0": {
        "Interface": "ap0",
        "Ethernet": {
            "State": "Enabled",
        },
    },
    "wlan0": {
        "Interface": "wlan0",
        "Ethernet": {
            "State": "Enabled",
        },
    },
}

STATUS_DATA = {
    "wlan": {
        "State": "",  # 状态信息
        "SSID": "",  # 连接名称
        # "Passphrase": "",  # 连接密码
        "Mask": "255.255.252.0",  # 子网掩码
        "Default Gateway": "",  # 网关
        "Band": "",  # 频段
        "RSSI": "",  # 信号强度
        # config
        "DHCP": "",  # ip分配
        "IP": "",  # ip地址
        "Bytes": "",
    },
    "ap": {
        "Band": "",  # 频段
        "SSID": "standard001",  # ap名称
        "Passphrase": "standard",  # ap密码
        "State": "Enabled",  # ap 状态
    },
    "Firmware Version": "",  # 状态信息
    "Mac": "",  # MAC地址  是ETH2的mac地址
    "speed": 138  # 速率
}
DOCTYPE = '''<!DOCTYPE configrecord [<!ELEMENT configrecord (configgroup+)>
                                    <!ELEMENT configgroup (configitem+)>
                                    <!ELEMENT configitem (value+)>
                                    <!ELEMENT value (#PCDATA)>
                                    <!ATTLIST configrecord version CDATA #IMPLIED>
                                    <!ATTLIST configgroup name CDATA #IMPLIED>
                                    <!ATTLIST configgroup instance CDATA #IMPLIED>
                                    <!ATTLIST configitem name CDATA #IMPLIED>
                                    <!ATTLIST configitem instance CDATA #IMPLIED>
                                    <!ATTLIST value name CDATA #IMPLIED>
                                    ]>'''
