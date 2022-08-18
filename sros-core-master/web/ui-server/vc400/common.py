# -*- coding:utf-8 -*-
# file common.py
# author YangHuaiDong
# date 2022/3/24 下午5:37
# copyright Copyright (c) 2021 Standard Robots Co., Ltd. All rights reserved.
# describe
from lxml import etree
from sros_log import _logger
from config import DOCTYPE, CONF_PATH, STATUS_PATH


def write_config_xml(tree, out_path):
    '''''将xml文件写出
      tree: xml树
      out_path: 写出路径'''
    tree.write(out_path, standalone="yes", method="xml", doctype=DOCTYPE, xml_declaration=True)

    with open(out_path, "r") as f:
        xml_bool = f.read()
    with open(out_path, "w") as t:
        t.write(xml_bool.replace("encoding='ASCII'", ""))


def modify_ap_parameter(ap_conf_path, AP_DATA):
    try:
        tree = etree.parse(ap_conf_path)
        for item in tree.xpath("//configgroup"):
            if item.attrib["name"] == "Access Point" and item.attrib["instance"] == "ap0":
                for child in item:
                    if child.attrib["name"] == "SSID":
                        child[0].text = AP_DATA["Access Point"]["SSID"]
                    elif child.attrib["name"] == "Passphrase":
                        child[0].text = AP_DATA["Access Point"]["Passphrase"]

            elif item.attrib["name"] == "Interface" and item.attrib["instance"] == "ap0":
                for child in item:
                    if child.attrib["name"] == "State":
                        child[0].text = AP_DATA["Interface_ap"]["State"]

                    elif child.attrib["name"] == "IP Address":
                        child[0].text = AP_DATA["Interface_ap"]["IP Address"]

                    elif child.attrib["name"] == "DHCP IP Address Range":
                        for sun in child:
                            if sun.attrib["name"] == "Start":
                                sun.text = AP_DATA["Interface_ap"]["DHCP IP Address Range"]["Start"]
                            elif sun.attrib["name"] == "End":
                                sun.text = AP_DATA["Interface_ap"]["DHCP IP Address Range"]["End"]

            elif item.attrib.get("name") == "HTTP Server":
                for configgroup_child in item:
                    if configgroup_child.attrib.get("name") == "Port":
                        configgroup_child[0].text = AP_DATA["HTTP Server"]["Port"]

            elif item.attrib["name"] == "Interface" and item.attrib["instance"] == "wlan0":
                for child in item:
                    if child.attrib["name"] == "State":
                        child[0].text = AP_DATA["Interface_wlan"]["State"]
        write_config_xml(tree, ap_conf_path)
    except Exception as e:
        _logger.error(e)
        return False
    return True


def modify_wlan_parameter(wlan_conf_path, WLAN_DATA):
    try:
        tree = etree.parse(wlan_conf_path)
        for configgroup in tree.xpath("//configgroup"):
            if configgroup.get("name") == 'WLAN Profile':
                configgroup.set("instance", WLAN_DATA["Basic"]["Network Name"])
                for configgroup_child in configgroup:
                    for configgroup_sun in configgroup_child:
                        if configgroup_sun.attrib.get("name") == "Network Name":
                            configgroup_sun.text = WLAN_DATA["Basic"]["Network Name"]
                        elif configgroup_sun.attrib.get("name") == "WPAx Passphrase":
                            configgroup_sun.text = WLAN_DATA["Basic"]["WPAx Passphrase"]

        write_config_xml(tree, wlan_conf_path)
    except Exception as e:
        _logger.error(e)
        return False
    return True


def modify_bridge(bridge_obj, bridge_conf_path, BRIDGE_DATA):
    """
    bridge_obj :桥接到 ap0 还是 wlan0
    """
    try:
        tree = etree.parse(bridge_conf_path)
        for item in tree.xpath("//configgroup"):
            if item.attrib["name"] == "Bridge":
                if bridge_obj == "ap0":
                    for child in item:
                        if child.attrib["name"] == "Interface":
                            child[0].text = bridge_obj  # interface

                        elif child.attrib["name"] == "Ethernet":
                            for sun in child:
                                if sun.attrib["name"] == "State":
                                    sun.text = BRIDGE_DATA["ap0"]["Ethernet"]["State"]

                elif bridge_obj == "wlan0":
                    for child in item:
                        if child.attrib["name"] == "Interface":
                            child[0].text = bridge_obj  # interface
                        elif child.attrib["name"] == "Ethernet":
                            for sun in child:
                                if sun.attrib["name"] == "State":
                                    sun.text = BRIDGE_DATA["wlan0"]["Ethernet"]["State"]

        write_config_xml(tree, bridge_conf_path)
    except Exception as e:
        _logger.error(e)
        return False
    return True


def modify_setting_ap(ap_state_path, state):
    """开启关闭ap state: Enabled/Disabled"""
    try:
        tree = etree.parse(ap_state_path)
        for item in tree.xpath("//configgroup"):
            if item.attrib["name"] == "Interface" and item.attrib["instance"] == "ap0":
                for child in item:
                    if child.attrib["name"] == "State":
                        child[0].text = state
        write_config_xml(tree, ap_state_path)
    except Exception as e:
        _logger.error(e)
        return False
    return True


def find_status(STATUS_DATA):
    try:
        tree = etree.parse(STATUS_PATH)
        for item in tree.xpath("//statusgroup"):
            if item.attrib["name"] == "WLAN" and item.attrib["instance"] == "wlan0":
                for child in item:
                    if child.attrib["name"] == "Connection State":
                        STATUS_DATA["wlan"]["State"] = child[0].text

                    elif child.attrib["name"] == "SSID":
                        STATUS_DATA["wlan"]["SSID"] = child[0].text

                    elif child.attrib["name"] == "Band":
                        STATUS_DATA["wlan"]["Band"] = child[0].text

                    elif child.attrib["name"] == "RSSI":
                        STATUS_DATA["wlan"]["RSSI"] = child[0].text

            elif item.attrib["name"] == "Access Point" and item.attrib["instance"] == "ap0":
                for child in item:
                    if child.attrib["name"] == "Band":
                        STATUS_DATA["ap"]["Band"] = child[0].text

                    elif child.attrib["name"] == "SSID":
                        STATUS_DATA["ap"]["SSID"] = child[0].text

            elif item.attrib["name"] == "Interface" and item.attrib["instance"] == "ap0":
                for child in item:
                    if child.attrib["name"] == "Status":
                        STATUS_DATA["ap"]["State"] = child[0].text

            elif item.attrib["name"] == "Interface" and item.attrib["instance"] == "wlan0":
                for child in item:
                    if child.attrib["name"] == "Current":
                        for su in child:
                            if su.attrib["name"] == "Default Gateway":
                                STATUS_DATA["wlan"]["Default Gateway"] = su.text

            elif item.attrib["name"] == "Device":
                for child in item:
                    if child.attrib["name"] == "Firmware Version":
                        STATUS_DATA["Firmware Version"] = child[0].text

            elif item.attrib["name"] == "Interface Counters" and item.attrib["instance"] == "wlan0":
                for child in item:
                    if child.attrib["name"] == "Receive":
                        for sun in child:
                            if sun.attrib["name"] == "Bytes":
                                STATUS_DATA["wlan"]["Bytes"] = sun.text

        if not find_conf(STATUS_DATA):
            _logger.error("find_conf is error")
            return False
    except Exception as e:
        _logger.error(e)
        return False
    return STATUS_DATA


def find_conf(STATUS_DATA):
    try:
        tree1 = etree.parse(CONF_PATH)
        for conf in tree1.xpath("//configgroup"):
            if conf.attrib["name"] == "Interface" and conf.attrib["instance"] == "wlan0":
                for child in conf:
                    if child.attrib["name"] == "DHCP Client":
                        STATUS_DATA["wlan"]["DHCP"] = child[0].text
                    elif child.attrib["name"] == "IP Address":
                        ip_and_mask = child[0].text
                        if "." in ip_and_mask:
                            ip_and_mask_list = ip_and_mask.split("/")
                            ip = ip_and_mask_list[0]
                            mask = ip_and_mask_list[1]
                            STATUS_DATA["wlan"]["IP"] = ip
                            STATUS_DATA["wlan"]["Mask"] = bit_length_to_netmask(int(mask))
    except Exception as e:
        _logger.error(e)
        return False
    return True


def import_conf_parameter(children_conf_path, conf_parameter):
    # 导入配置参数进网卡配置
    try:
        tree1 = etree.parse(children_conf_path)
        for conf in tree1.xpath("//configgroup"):
            if conf.attrib["name"] == "Interface" and conf.attrib["instance"] == "wlan0":
                for child in conf:
                    if child.attrib["name"] == "State":
                        child[0].text = conf_parameter["wlan"]["State"]

                    elif child.attrib["name"] == "DHCP Client":
                        if conf_parameter["wlan"]["DHCP"] == "":
                            continue
                        child[0].text = conf_parameter["wlan"]["DHCP"]

                    elif child.attrib["name"] == "IP Address":
                        if conf_parameter["wlan"]["IP"] == "" or conf_parameter["wlan"]["Mask"] == "":
                            continue
                        mask_int = netmask_to_bit_length(conf_parameter["wlan"]["Mask"])
                        child[0].text = conf_parameter["wlan"]["IP"] + "/" + str(mask_int)

                    elif child.attrib["name"] == "Default Gateway":
                        if conf_parameter["wlan"]["DHCP"] == "Enabled":
                            continue
                        else:
                            child[0].text = conf_parameter["wlan"]["Default Gateway"]

            elif conf.attrib["name"] == "Interface" and conf.attrib["instance"] == "ap0":
                for child in conf:
                    if conf_parameter["ap"]["State"] == "":
                        continue
                    elif child.attrib["name"] == "State":
                        child[0].text = conf_parameter["ap"]["State"]

            elif conf.attrib["name"] == "Access Point" and conf.attrib["instance"] == "ap0":
                for child in conf:
                    if child.attrib["name"] == "SSID":
                        if conf_parameter["ap"]["SSID"] == "":
                            continue
                        child[0].text = conf_parameter["ap"]["SSID"]

                    elif child.attrib["name"] == "Passphrase":
                        if conf_parameter["ap"]["Passphrase"] == "":
                            continue
                        child[0].text = conf_parameter["ap"]["Passphrase"]

            elif conf.attrib["name"] == "Radio":
                for child in conf:
                    if conf_parameter["ap"]["Band"] == "":
                        continue
                    elif child.attrib["name"] == "Band":
                        child[0].text = conf_parameter["ap"]["Band"] + " Only"

        write_config_xml(tree1, children_conf_path)
    except Exception as e:
        _logger.error(e)
        return False
    return True


# 子网掩码地址转长度
def netmask_to_bit_length(netmask):
    # 分割字符串格式的子网掩码为四段列表
    # 计算二进制字符串中 '1' 的个数
    # 转换各段子网掩码为二进制, 计算十进制
    return sum([bin(int(i)).count('1') for i in netmask.split('.')])


# 子网掩码长度转地址
def bit_length_to_netmask(mask_int):
    bin_array = ["1"] * mask_int + ["0"] * (32 - mask_int)
    tmpmask = [''.join(bin_array[i * 8:i * 8 + 8]) for i in range(4)]
    tmpmask = [str(int(netmask, 2)) for netmask in tmpmask]
    return '.'.join(tmpmask)
