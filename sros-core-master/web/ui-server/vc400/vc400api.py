# -*- coding: utf-8 -*-
# !/usr/bin/python
import json
import os
import time

import web

from ap_conf_script import ApConfigScript
from common import netmask_to_bit_length
from vc400class import VC400Class
from config import (
    WLAN_CONF_PATH,
    BRIDGE_CONF_PATH,
    AP_CONF_PATH,
    STATUS_DATA,
    NETWORK_WLAN_DATA,
    BRIDGE_DATA,
)
from sros_log import _logger

db_main = web.database(dbn='sqlite', db="/sros/db/main.db3")
table_name_vc400 = 'vc400'


class API_VC400_DoNetworkScan():
    def GET(self):
        return json.dumps(VC400Class().do_network_scan())


class API_VC400_AllExportConfigXml():
    def GET(self):
        return json.dumps(VC400Class().all_export_config_xml())


class API_VC400_ImportConfigXml():
    def POST(self, conf_path):
        res = VC400Class().import_config_xml(conf_path)
        return json.dumps(res)


class API_VC400_DeleteNetwork():
    def POST(self, delete_wlan_path, wlan_name):
        res = VC400Class().delete_network(delete_wlan_path, wlan_name)
        return json.dumps(res)

    def OPTIONS(self):
        pass


class API_VC400_RebootDevice():
    def GET(self):
        res = VC400Class().reboot_device()
        return json.dumps(res)


class API_VC400_FactoryDefaults():
    def GET(self):
        res = VC400Class().factory_defaults()
        return json.dumps(res)


class API_VC400_PushWlanConf():
    def POST(self):
        dat = json.loads(web.data())
        wlan_data = dat.get("wlan_data")
        res = VC400Class().push_wlan_conf(WLAN_CONF_PATH, wlan_data)
        return json.dumps({"code": 200 if res else 400})

    def OPTIONS(self):
        pass


class API_VC400_PushApConf():
    def POST(self):
        dat = json.loads(web.data())
        ap_data = dat.get("ap_data")
        res = ApConfigScript().push_ap_conf(
            AP_CONF_PATH,
            BRIDGE_CONF_PATH,
            ap_data,
            BRIDGE_DATA
        )
        if os.system("sh /sros/web/ui-server/vc400/sh_package/ap_conf.sh") != 0:
            return json.dumps({"code": 400})
        return json.dumps({"code": 200 if res else 400})

    def OPTIONS(self):
        pass


class API_VC400_FindStatus():
    def GET(self):
        ## STATUS_DATA :  还是从网卡配置状态获取
        res = VC400Class().find_status(STATUS_DATA)
        return json.dumps(res)


class API_VC400_ImportConf():
    def POST(self):
        # 前端接收 推给网卡
        dat = json.loads(web.data())
        conf_data = dat.get("conf_data")
        # check data
        new_conf_data = self.check_ap_and_wlan(conf_data)
        # todo 存人数据库
        try:
            vc400_id = db_main.insert(
                table_name_vc400,
                ap=str(new_conf_data["ap"]), wlan=str(new_conf_data["wlan"]),
                create_time=int(time.time()),
            )
        except Exception as e:
            _logger.error(e)
            return json.dumps({"code": 400, "error_msg": e})
        if not vc400_id:
            _logger.info("import vc400-wifi setting is error")
            return json.dumps({"code": 400, "error_msg": "import vc400-wifi setting is error"})

        res = VC400Class().import_conf_paramter(new_conf_data, BRIDGE_CONF_PATH, BRIDGE_DATA)
        if not res:
            return json.dumps({"code": 400})

        if new_conf_data["wlan"]["State"] == "Enabled":
            if self.check_dhcp_status(new_conf_data) != True:
                _logger.info("check_dhcp_status is error")
                return json.dumps({"code": 400, "error_msg": "check_dhcp_status is error"})
            if os.system("sh /sros/web/ui-server/vc400/sh_package/wlan_conf.sh") != 0:
                return json.dumps({"code": 400, "error_msg": "wlan_conf.sh is error"})
        else:
            if os.system("sh /sros/web/ui-server/vc400/sh_package/ap_conf.sh") != 0:
                return json.dumps({"code": 400, "error_msg": "ap_conf.sh is error"})
        os.system("systemctl restart sros")

        return json.dumps({"code": 200})

    def check_dhcp_status(self, conf_data):
        dhcp_status = conf_data["wlan"].get("DHCP")
        if dhcp_status == "Disabled":
            NETWORK_WLAN_DATA["DHCP"] = "no"
            mask_int = netmask_to_bit_length(conf_data["wlan"]["Mask"])
            NETWORK_WLAN_DATA["Address"] = conf_data["wlan"].get("IP") + "/" + str(mask_int)
            NETWORK_WLAN_DATA["Gateway"] = conf_data["wlan"].get("Default Gateway")
        else:
            NETWORK_WLAN_DATA["DHCP"] = "yes"
            if NETWORK_WLAN_DATA.get("Address") != None:
                del NETWORK_WLAN_DATA["Address"], NETWORK_WLAN_DATA["Gateway"]

        if not VC400Class().modify_eth2_info(NETWORK_WLAN_DATA):
            return json.dumps({"code": 400, "error_msg": "modify_eth2_info is error"})

        return True

    def check_ap_and_wlan(self, conf_data):
        if conf_data["ap"]["State"] == "Link up":
            conf_data["ap"]["State"] = "Enabled"
        else:
            conf_data["ap"]["State"] = "Disabled"

        if conf_data["wlan"]["State"] == "Connected":
            conf_data["wlan"]["State"] = "Enabled"
        else:
            conf_data["wlan"]["State"] = "Disabled"

        return conf_data

    def OPTIONS(self):
        pass
