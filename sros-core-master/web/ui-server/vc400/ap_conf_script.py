# -*- coding:utf-8 -*-
# file ap_conf_script.py
# author YangHuaiDong
# date 2022/2/22 下午1:35
# copyright Copyright (c) 2021 Standard Robots Co., Ltd. All rights reserved.
# describe
import sqlite3
import time
import traceback
import requests
from lxml import etree
from requests.auth import HTTPDigestAuth

from common import modify_ap_parameter, modify_bridge
from config import (
    DB_PATH,
    AP_CONF_PATH,
    BRIDGE_CONF_PATH,
    AP_DATA,
    BRIDGE_DATA,
)
from sros_log import _logger


class ApConfigScript():

    def __init__(self):
        self.db = sqlite3.connect(DB_PATH)
        self.cur = self.db.cursor()
        self.retry = 3

        self.ip = "169.254.0.1"
        self.auth = HTTPDigestAuth("admin", "PASSWORD")
        self.import_conf_url = "http://{}/import/config".format(self.ip)
        self.export_conf_url = "http://{}/export/config".format(self.ip)

    def get_config_value(self, key):
        self.retry = 3

        while self.retry > 0:
            try:
                self.cur.execute("SELECT value FROM config where key = '%s' and is_valid=1;" % key)
                rs = self.cur.fetchone()
                if rs is not None and len(rs) != 0:
                    return rs[0].encode('ascii', 'ignore')
                else:
                    return ""
            except:
                traceback.print_exc()
                self.retry = self.retry - 1
                time.sleep(0.1)

        return ""

    def find_vehicle_serial_no(self):
        # 查找机器序列号
        if self.get_config_value("main.vehicle_serial_no") == b"NA":
            _logger.info("find_vehicle_serial_no is empty")
            return False
        else:
            vehicle_serial_no = bytes.decode(self.get_config_value("main.vehicle_serial_no"))
            _logger.info("find_vehicle_serial_no %s" % vehicle_serial_no[-8:])
            return vehicle_serial_no[-8:]

    def import_config_xml(self, conf_path):
        """导入配置文件"""
        with open(conf_path, "r") as f:
            f_read = f.read()
        files = {"configrecord": f_read}
        res = requests.post(url=self.import_conf_url, files=files, auth=self.auth, timeout=50)
        # config_xml = str(res.content, encoding="utf-8")
        config_xml = str(res.content)
        root = etree.XML(config_xml)
        if root.xpath("//result")[0].text != "Succeeded":
            _logger.error(root.xpath("//result")[0].text)
            return False
        return True

    def push_ap_conf(self, config_xml_path, bridge_xml_path, AP_DATA, BRIDGE_DATA):
        serial_no = self.find_vehicle_serial_no()
        if not serial_no:
            code = modify_ap_parameter(config_xml_path, AP_DATA)
        else:
            AP_DATA["Access Point"]["SSID"] = serial_no
            code = modify_ap_parameter(config_xml_path, AP_DATA)
        if not code:
            _logger.error("modify_ap_parameter is error!")
            return False
        if not self.import_config_xml(config_xml_path):
            _logger.error("push_ap_conf import wlan conf is error!")
            return False
        # TODO 添加检验条件 检查wlan0是否连接上 连上之后才可以改变桥接
        #
        if not modify_bridge("ap0", bridge_xml_path, BRIDGE_DATA):
            _logger.error("push_ap_conf modify_bridge is error!")
            return False
        if not self.import_config_xml(bridge_xml_path):
            _logger.error("push_ap_conf import bridge conf is error!")
            return False
        _logger.info("push_ap_conf is successful!!!")
        return True


if __name__ == '__main__':
    print("push_ap_conf is successful!!!" + str(
        ApConfigScript().push_ap_conf(AP_CONF_PATH, BRIDGE_CONF_PATH, AP_DATA, BRIDGE_DATA)))
