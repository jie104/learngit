# -*- coding:utf-8 -*-
import socket
import struct
# import fcntl

import os
import configparser
import time
import web
from lxml import etree
import requests
from requests.auth import HTTPDigestAuth
from common import (
    write_config_xml,
    modify_wlan_parameter,
    modify_bridge,
    modify_setting_ap,
    find_status,
    import_conf_parameter,
)
from sros_log import _logger
from config import (
    CONF_PATH,
    STATUS_PATH,
    AP_STATE_PATH,
    NETWORK_WLAN_PATH,
    CHILREN_CONF_PATH
)

db_main = web.database(dbn='sqlite', db="/sros/db/main.db3")
table_name_vc400 = 'vc400'


class MyConfigParser(configparser.ConfigParser):
    """
    set ConfigParser options for case sensitive.
    """

    def __init__(self, defaults=None):
        configparser.ConfigParser.__init__(self, defaults=defaults)

    def optionxform(self, optionstr):
        return optionstr


class VC400Class(object):
    def __init__(self):
        self.ip = "169.254.0.1"
        self.port = 8080
        self.auth = HTTPDigestAuth("admin", "PASSWORD")

        # 导出配置文件url
        self.export_conf_url = "http://{0}:{1}/export/config".format(self.ip, self.port)
        # 导入配置文件url
        self.import_conf_url = "http://{0}:{1}/import/config".format(self.ip, self.port)
        # 导出状态文件url
        self.export_status_url = "http://{0}:{1}/export/status".format(self.ip, self.port)
        # 采取状态行动
        self.action_status_url = "http://{0}:{1}/action/status".format(self.ip, self.port)
        self.embedded_url = "http://{}:{}/fs/embedded".format(self.ip, self.port)

        self.speed = 0

    def all_export_config_xml(self):
        """导出配置文件--->全部导出"""
        res = requests.post(url=self.export_conf_url, auth=self.auth)
        with open(CONF_PATH, "wb") as f:
            f.write(res.content)
        return {"code": res.status_code}

    def import_config_xml(self, conf_path):
        """导入配置文件"""
        with open(conf_path, "r") as f:
            f_read = f.read()
        files = {"configrecord": f_read}
        res = requests.post(url=self.import_conf_url, files=files, auth=self.auth, timeout=300)
        # config_xml = str(res.content, encoding="utf-8")
        config_xml = str(res.content)
        root = etree.XML(config_xml)
        if root.xpath("//result")[0].text != "Succeeded":
            _logger.error(root.xpath("//result")[0].text)
            return False
        return True

    def all_export_status_xml(self):
        """导出状态文件"""
        res = requests.post(url=self.export_status_url, auth=self.auth)
        with open(STATUS_PATH, "wb") as f:
            f.write(res.content)
        return {"code": res.status_code}

    def action_status(self):
        """采取状态行动api
        组：必填。定义操作的状态组。
        optionalGroupInstance：状态组的可选实例。
        optionalItem：定义动作的状态组的可选项目。
        optionalItemInstance：状态项的可选实例。
        需要采取的行动。要采取的行动。有关操作列表，请参阅操作定义。
        """
        pass

    def portion_export_config_clock_xml(self):
        """导出配置文件--->局部导出例子"""
        data = {"optionalGroupList": "Device;Clock"}
        res = requests.post(url=self.export_conf_url, auth=self.auth, data=data, timeout=300)
        # portion_clock_xml = str(res.content, encoding="utf-8")
        portion_clock_xml = str(res.content)
        return portion_clock_xml

    def do_network_scan(self):
        """扫描wlan"""
        data = {
            "group": "WLAN",
            "optionalGroupInstance": "wlan0",
            "action": "Broadcast Scan",
        }
        res = requests.post(self.action_status_url, data=data, auth=self.auth, timeout=300)
        root = etree.XML(res.content)
        wlan_list = list()
        total = 0
        base_wlan_template = dict()
        for item in root.xpath("//statusitem"):
            if item.attrib["name"] == "Number of responses":
                total = item[0].text
            else:
                next_data = base_wlan_template.copy()
                next_data["SSID"] = item[0].text
                next_data["BSSID"] = item[1].text
                next_data["Channel"] = item[2].text
                next_data["RSSI"] = item[3].text
                next_data["Security Suite"] = item[5].text
                next_data["Is Active"] = item[6].text
                wlan_list.append(next_data)

        return {"wlan_list": wlan_list, "total": total}

    def delete_network(self, delete_wlan_path, wlan_name="SR_office"):
        """删除wlan配置"""
        # delete_wlan_path = "xml_package/delete_wlan.xml"
        tree = etree.parse(delete_wlan_path)
        for configitem in tree.xpath("//configitem"):
            for configitem_child in configitem:
                if configitem_child.get("name") == 'name':
                    configitem_child.text = wlan_name

        write_config_xml(tree, delete_wlan_path)
        if not self.import_config_xml(delete_wlan_path):
            _logger.info("delete_network import delete conf is error")
            return False
        return True

    def reboot_device(self):
        """重启设备"""
        data = {"group": "Device", "action": "Reboot", }
        res = requests.post(self.action_status_url, data=data, auth=self.auth, timeout=300)
        if res.status_code != 200:
            _logger.info("reboot_device is error")
            return {"code": 400}
        # self.reboot_asix()
        return {"code": res.status_code}

    def reboot_asix(self):
        # 重启或者恢复出厂设置的同时 要安装卸载网卡
        os.system("rmmod asix.ko")
        time.sleep(2)
        os.system("insmod asix.ko")

    def factory_defaults(self):
        """恢复出厂设置"""
        data = {"group": "Device", "action": "Factory Defaults", }
        res = requests.post(self.action_status_url, data=data, auth=self.auth, timeout=300)
        self.reboot_asix()
        if res.status_code != 200:
            _logger.info("factory_defaults is error")
            return {"code": 400}
        return {"code": 200}

    def modify_eth2_info(self, eth2_data):
        """修改eth2网卡信息CRUD"""
        # eth2 = {"DHCP": "no", "Address": "192.168.0.5/22", "Gateway": "192.168.0.1"},
        conf = MyConfigParser()
        try:
            conf.read(NETWORK_WLAN_PATH)
            conf.remove_section("Match")
            conf.remove_section("Network")
            conf["Match"] = {"Name": "eth2"}
            conf["Network"] = eth2_data
            conf.write(open(NETWORK_WLAN_PATH, "w"), space_around_delimiters=False)
            # 修改之后要重启网卡生效
            # os.system("systemctl restart systemd-networkd")
        except Exception as e:
            _logger.error(e)
            return False
        return True

    def export_wlan_setting(self):
        """网络设置"""
        from_data = {
            "optionalGroupList": "Interface:eth0;Interface: ap0;Interface: wlan0;User: admin;Access Point: ap0;Bridge"
        }

        res = requests.post(self.export_conf_url, data=from_data, auth=self.auth, timeout=300)
        # 接收返回网络设置的wlan xml
        # wlan_set_xml = str(res.content, encoding="utf-8")
        wlan_set_xml = str(res.content)

        if res.status_code != 200:
            _logger.info("export_wlan_setting is error code~")
            return {"code": 400}
        return res.status_code

    def populate_status(self):
        from_data = {
            "optionalGroupList": "Access Point;Interface;WLAN",
            # "optionalGroupList":"WLAN",
        }

        res = requests.post(self.export_status_url, data=from_data, auth=self.auth, timeout=300)
        # 接收返回网络设置的wlan xml
        wlan_status_xml = str(res.content)

        if res.status_code != 200:
            raise Exception("error code 2222")
        return wlan_status_xml

    def push_wlan_conf(self, wlan_conf_path, WLAN_DATA):
        """推送wlan配置"""
        res = modify_wlan_parameter(wlan_conf_path, WLAN_DATA)
        if not res:
            _logger.info("modify_wlan_parameter is error!")
            return False
        if not self.import_config_xml(wlan_conf_path):
            _logger.info("push_wlan_conf import wlan conf is error!")
            return False
        return True

    def get_mac_address(self, if_name):
        try:
            mac = open('/sys/class/net/' + if_name + '/address').readline().strip().upper()
        except:
            mac = "00:00:00:00:00:00"

        return mac

    def check_ping(self, ip):
        """检查是否能ping通"""
        # ip = '10.10.70.1'
        backinfo = os.system('ping -c 1 -w 1 %s ' % ip)
        return False if backinfo else True

    def modify_setting_ap(self, state):
        res = modify_setting_ap(AP_STATE_PATH, state)
        if not res:
            _logger.info("modify_setting_ap is error!")
            return False
        if not self.import_config_xml(AP_STATE_PATH):
            _logger.info("down_or_up_ap import conf is error!")
            return False
        return True

    def find_status(self, STATUS_DATA):
        if self.all_export_status_xml().get("code") != 200:
            _logger.error("find_status export status is error")
            return

        if self.all_export_config_xml().get("code") != 200:
            _logger.error("find_status export config is error")
            return

        new_status = find_status(STATUS_DATA)

        byte = round(int(new_status.get("wlan").get("Bytes")) / float((1024 * 1024)), 4)
        new_status["wlan"]["Bytes"] = byte
        _logger.info(new_status)
        if self.speed == 0:
            new_status["speed"] = self.speed
        else:
            speed = round((byte - self.speed) / float(60), 4)
            new_status["speed"] = speed
        self.speed = byte
        _logger.info(self.speed)
        new_status["Mac"] = self.get_mac_address("eth2")
        if self.get_ap_passwd():
            new_status["ap"]["Passphrase"] = self.get_ap_passwd()
        return new_status

    def get_ap_passwd(self):
        try:
            rs = db_main.select(table_name_vc400, order='create_time DESC')
            if not rs:
                return None
            for i in rs:
                pa = eval(i["ap"])
                return pa["Passphrase"]
        except BaseException as e:
            print(e)

    def import_conf_paramter(self, conf_data, bridge_conf_path, BRIDGE_DATA):
        if not import_conf_parameter(CHILREN_CONF_PATH, conf_data):
            _logger.error("import_conf_parameter is error")
            return False

        if not self.import_config_xml(CHILREN_CONF_PATH):
            _logger.info("push_wlan_conf import wlan conf is error!")
            return False

        if conf_data["wlan"]["State"] == "Enabled":
            if not modify_bridge("wlan0", bridge_conf_path, BRIDGE_DATA):
                _logger.info("import_conf_paramter modify_bridge wlano is error!")
                return False
        else:
            if not modify_bridge("ap0", bridge_conf_path, BRIDGE_DATA):
                _logger.info("import_conf_paramter modify_bridge ap0 is error!")
                return False

        if not self.import_config_xml(bridge_conf_path):
            _logger.info("push_wlan_conf import bridge conf is error!")
            return False

        return True
