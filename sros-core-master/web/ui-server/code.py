# -*- coding: utf-8 -*- 
# !/usr/bin/python

import cgi
import copy
import csv
import re
import sys
from utils import *
from itertools import islice
import sqlite3

import web
from data_parser import (
    LMKParser,
    MonitorParser,
    QRCodeParser,
)

from database import (
    db_main as db,
    ConfigAdmin,
    FixedRecord,
    MapsAdmin,
    MissionRecord,
    ScheduleAdmin,
    TaskTemplateAdmin,
    UpgradeRecord,
    UserMissionAdmin,
)

from account_admin import (
    API_AccountAdmin,
    API_Account,
)

from config_admin import API_ConfigAdmin
from generate_maps_db import generate_maps_db_data
from sros_log import _logger

# 无线网卡相关代码暂时屏蔽
#from vc400.vc400api import (
#    API_VC400_DoNetworkScan,
#    API_VC400_RebootDevice,
#    API_VC400_PushWlanConf,
#    API_VC400_PushApConf,
#    API_VC400_FindStatus,
#    API_VC400_ImportConf,
#)

url = ''

render = web.template.render('templates/')

reload(sys)
sys.setdefaultencoding('utf-8')

# 解决管道通信时，一段时间后出现broken pipe问题
# [Errno 32] Broken pipe
# https://blog.csdn.net/woay2008/article/details/83592298

# 网站基本信息配置
config = web.storage(
    # 网站名称，显示在导航栏的LOGO位置
    site_name=U"SROS Admin",
    # 网站描述，显示在网页窗口标题的位置
    site_desc=U'SROS admin page',
    # 以下三项请勿修改
    root=url + '',
    refer=url + '/',
    static=url + '/static',
)

cgi.maxlen = 100 * 1024 * 1024  # max upload file size = 100MB

# 数据库连接配置，host=数据库服务器地址，user=用户名，pw=密码，db=数据库名称
db = web.database(dbn='sqlite', db="/sros/db/main.db3")
monitor_parser = MonitorParser()

BUF_SIZE = 1024
SROS_MAP_DIR = "/sros/map/"
SROS_TOOL_DIR = "/sros/tool/"

web.template.Template.globals['render'] = render
web.template.Template.globals['config'] = config

# route
urls = (
    '/', 'index',
    '/setting', 'SettingList',
    '/map', 'MapList',
    '/update', 'UpdateFirmware',

    '/camera/preview', 'CameraPreview',

    '/api/v0/setting', 'API_SettingList',
    '/api/v0/setting/(\d+)', 'API_SettingItem',

    '/api/v0/db', 'API_DB',

    '/api/v0/map', 'API_MapList',
    '/api/v0/map/(.+)/data', 'API_MapItem',
    '/api/v0/map/(.+)/meta', 'API_MapInfo',
    '/api/v0/map/(.+)/update', 'API_MAPItemUpdate',
    '/api/v0/map/(.+)/image', 'API_MapItemImage',
    '/api/v0/map/(.+)/export', 'API_MapExport',
    '/api/v0/map/(.+)/pack', 'API_MapExport',
    '/api/v0/map/import', 'API_MapImport',
    '/api/v0/map/(.+)/to/(.+)', 'API_MapCopyPath',

    '/api/v1/map/get_list', 'API_V1_MapList',
    '/api/v1/map/update/(.+)/(.+)/target/(.+)', 'API_V1_MapList',

    # 优化地图列表切换不流畅,新增几个接口
    '/api/v2/map', 'API_V2_MapList',
    '/api/v2/map/(.+)', 'API_V2_MapList',

    '/api/v0/mission/import', 'API_TaskImport',
    '/api/v0/mission/(\d+)/export', 'API_TaskExport',

    '/api/v0/mission_template', 'API_TaskTemplate',
    '/api/v0/mission_template/(\d+)/delete', 'API_TaskTemplate',
    '/api/v0/mission_template/import', 'API_TaskTemplateImport',
    '/api/v0/mission_template/(\d+)/export', 'API_TaskTemplateExport',

    '/api/v0/mission_record/clear', 'API_MissionRecordClear',

    # 定时任务/modbus/IO触发等计划任务
    '/api/v0/schedule/get', 'API_Schedule',
    '/api/v0/schedule/create', 'API_Schedule',
    '/api/v0/schedule/(.+)/update', 'API_Schedule',
    '/api/v0/schedule/(.+)/delete', 'API_Schedule',

    '/api/v0/maintenance/get', 'API_FixedRecord',
    '/api/v0/maintenance/create', 'API_FixedRecord',

    '/api/v0/camera/(\d+)/now', 'API_CameraNow',
    '/api/v0/vision/(\d+)/now', 'API_VisionNow',
    '/api/v0/mission/(\d+)', 'API_Mission',
    '/api/v0/(\d+)/missions', 'API_MissionList',
    '/api/v0/log/export', 'API_ClassicLogExport',
    '/api/v0/log/startup', 'API_RunLogExport',
    '/api/v0/userlog/follow', 'API_ClassicUserLogFollow',
    '/api/v0/userlog/section', 'API_ClassicUserLogSection',
    '/api/v0/update/import', 'API_SystemUpdateImport',
    '/api/v0/src/import', 'API_SrcUpdateImport',
    '/api/v0/rollback/system', 'API_SystemUpgradeRollback',

    '/api/v0/accounts', 'API_AccountAdmin',
    '/api/v0/account/(.+)', 'API_Account',

    '/api/v0/performance/get', 'API_Performance',
    '/api/v0/lmk/get', 'API_LMK',
    '/api/v0/qrcode/(.+)/get', 'API_QRCode',
    '/api/v0/qrcode/update', 'API_QRCode',

    '/api/v0/upgrade_record/system', 'API_SrosUpgradeRecord',
    '/api/v0/upgrade_record/src', 'API_SrcUpgradeRecord',

    '/api/v0/config_info/rollback', 'API_ConfigInfoRollback',
    '/api/v0/config_info/import', 'API_ConfigInfoImport',
    '/api/v0/config_info/(.+)/export', 'API_ConfigInfoExport',
    '/api/v0/configs/get', 'API_ConfigAdmin',
    '/api/v0/file/get', 'API_File',

    # 自定义logo文件接口
    '/api/v0/logo_customize', 'API_LogoCustomize',
    # TODO 无线网卡
    '/api/v0/vc400/do_network_scan', 'API_VC400_DoNetworkScan',
    '/api/v0/vc400/find_status', 'API_VC400_FindStatus',
    '/api/v0/vc400/reboot_device', 'API_VC400_RebootDevice',
    '/api/v0/vc400/push_ap_conf', 'API_VC400_PushApConf',
    '/api/v0/vc400/push_wlan_conf', 'API_VC400_PushWlanConf',
    '/api/v0/vc400/import_conf', 'API_VC400_ImportConf',
    # 配置参数重构参数映射接口
    '/api/v0/conf_parameter', 'API_ConfParameter',
    # 添加FMS校验地图文件MD5接口
    '/api/v0/check_map_md5', 'API_CheckMapMd5',
    # 提供相机标定板参数接口
    '/api/v0/calibration_conf', 'API_Calibration',
    # 告警管理接口
    '/api/v2/error_log/get', 'API_Error_Log',

    # NOTE: 解决静态资源重定向的问题，chip-web中遇到的，我们渲染index.html，但其中需要访问的静态文件都没有以static开头，导致访问不了。现在已这种别扭的方式解决
    '/js/(.+)', 'JS',
    '/fonts/(.+)', 'FONTS',
    '/img/(.+)', 'StaticFiles',
    '/statics/(.+)', 'StaticFiles',
    '/(.+)', 'CSS'
)


class API_Calibration():
    # 提供相机标定板参数接口
    def GET(self):
        with open("configs/calibration_conf.json", "r") as f:
            cal_conf = json.loads(f.read())
        return json.dumps({"data": cal_conf})


class API_CheckMapMd5():
    # 添加FMS校验地图文件MD5接口
    def POST(self):
        dat = json.loads(web.data())
        map_name = dat.get("map_name")
        file_path = "/tmp/%s_nav.map_export" % map_name
        with open(file_path, 'rb') as f:
            d = hashlib.md5()
            d.update(f.read())
            md5 = d.hexdigest()
        return json.dumps({"MD5": md5, "code": 200})


class API_ConfParameter():
    # 参数配置映射
    def __init__(self):
        (self.one_th_menu_name_dict,
         self.two_th_menu_name_dict,
         self.three_th_menu_name_dict,
         self.four_th_menu_name_dict,
         self.assembly_dict,
         self.new_dict) = dict(), dict(), dict(), dict(), dict(), dict()
        self.total_data = list()
        # 参数字段名
        self.conf_fields = ["1th_menu", "2th_menu", "3th_menu", "4th_menu",
                            "ItemID", "ValCheckFlag", "IsCommon", "VehicleType"]

    # 参数配置映射
    def GET(self):
        try:
            with open("configs/menu_id_mapping.csv", "r") as csv_menu_file:
                reader = csv.DictReader(csv_menu_file)
                # field_list = reader.fieldnames
                for row in reader:
                    if row["1th_menu_name"] != "":
                        self.one_th_menu_name_dict[row["1th_menu_id"]] = row["1th_menu_name"]
                        self.assembly_dict["1th_menu_name"] = self.one_th_menu_name_dict
                    if row["2th_menu_name"] != "":
                        self.two_th_menu_name_dict[row["2th_menu_id"]] = row["2th_menu_name"]
                        self.assembly_dict["2th_menu_name"] = self.two_th_menu_name_dict
                    if row["3th_menu_name"] != "":
                        self.three_th_menu_name_dict[row["3th_menu_id"]] = row["3th_menu_name"]
                        self.assembly_dict["3th_menu_name"] = self.three_th_menu_name_dict
                    if row["4th_menu_name"] != "":
                        self.four_th_menu_name_dict[row["4th_menu_id"]] = row["4th_menu_name"]
                        self.assembly_dict["4th_menu_name"] = self.four_th_menu_name_dict
            with open("configs/config_parameter_mapping.csv", "r") as csv_mapping_file:
                mapping_reader = csv.DictReader(csv_mapping_file)
                for mapping_item in mapping_reader:
                    new_dict = self.new_dict.copy()
                    if mapping_item["1th_menu"] != "":
                        mapping_item["1th_menu"] = self.assembly_dict["1th_menu_name"][mapping_item["1th_menu"]]
                    if mapping_item["2th_menu"] != "":
                        mapping_item["2th_menu"] = self.assembly_dict["2th_menu_name"][mapping_item["2th_menu"]]
                    if mapping_item["3th_menu"] != "":
                        mapping_item["3th_menu"] = self.assembly_dict["3th_menu_name"][mapping_item["3th_menu"]]
                    if mapping_item["4th_menu"] != "":
                        mapping_item["4th_menu"] = self.assembly_dict["4th_menu_name"][mapping_item["4th_menu"]]
                    for item in self.conf_fields:
                        if item == "ItemID" and mapping_item["ItemID"] != "xxxx" and mapping_item["ItemID"] != '':
                            new_dict["ItemID"] = int(mapping_item["ItemID"])
                        elif item == "IsCommon" and mapping_item["IsCommon"] != '':
                            new_dict["IsCommon"] = int(mapping_item["IsCommon"])
                        elif item == "ValCheckFlag" and mapping_item["ValCheckFlag"] != '':
                            new_dict["ValCheckFlag"] = int(mapping_item["ValCheckFlag"])
                        elif item == "VehicleType" and mapping_item["VehicleType"] != '':
                            new_dict["VehicleType"] = int(mapping_item["VehicleType"])
                        else:
                            new_dict[item] = mapping_item[item]
                    self.total_data.append(new_dict)
            return json.dumps({"data": self.total_data, "level": self.assembly_dict})
        except Exception as e:
            _logger.error("API_ConfParameter parameter is error: %s" % e)
            raise web.internalerror("parameter is error %s" % e)


class API_LogoCustomize:
    def __init__(self):
        self.IMPORT_LOGO_FILE_PATH = os.path.join(os.path.dirname(__file__), "static", "source/")
        self.files_list = ['favicon.ico', 'homeImage.svg', 'loginLogo.svg', 'companyInformation.json']

    def GET(self):
        pass

    def POST(self):
        # 自定义logo文件
        if not os.path.exists(self.IMPORT_LOGO_FILE_PATH):
            os.system("mkdir %s" % self.IMPORT_LOGO_FILE_PATH)

        logo_file_obj = web.input(file={})
        if 'file' in logo_file_obj:
            with open(self.IMPORT_LOGO_FILE_PATH + logo_file_obj['file'].filename, 'wb') as tmp_file:
                tmp_file.write(logo_file_obj['file'].file.read())

        os.system("tar zxvf %s -C %s" % (self.IMPORT_LOGO_FILE_PATH + logo_file_obj['file'].filename, "/tmp/"))
        os.system("rm %s" % (self.IMPORT_LOGO_FILE_PATH + logo_file_obj['file'].filename))
        for root, dir, files in os.walk("/tmp/companyInformation/"):
            if len(files) >= 4 and set(self.files_list) == set(files):
                os.system("rm -rf %s" % (self.IMPORT_LOGO_FILE_PATH + "companyInformation/"))
                os.system("mv -f /tmp/companyInformation/ %s" % self.IMPORT_LOGO_FILE_PATH)
                return json.dumps({"state": True, "code": 200, "message": "upload logo is successful"})

        return web.BadRequest()

    def DELETE(self):
        pass

    def OPTIONS(self):
        pass


class index:
    def GET(self):
        # return web.seeother('static/chip-web/spa-mat/index.html')
        return render.index()


class CSS:
    def GET(self, name):
        # print(name)
        if name.endswith('.css'):
            return web.seeother('/static/chip-web/spa-mat/' + name)
        else:
            # raise web.NotFound()
            return web.seeother('/')


class FONTS:
    def GET(self, name):
        return web.seeother('/static/chip-web/spa-mat/fonts/' + name)


class JS:
    def GET(self, name):
        # print 'static/js/' + name
        return web.seeother('/static/chip-web/spa-mat/js/' + name)


class StaticFiles:
    def GET(self, name):
        if name.endswith('.jpg') or name.endswith('.png') or name.endswith('.jpeg') \
                or name.endswith('.svg') or name.endswith('.html'):
            return web.seeother('/static/chip-web/spa-mat/statics/' + name)
        else:
            return web.seeother('/')


class SettingList:
    @authenticated
    def GET(self):
        ds = get_settings_list()
        return render.settings('settings', ds)


class CameraPreview:
    def GET(self):
        return render.camera_preview('camera_preview')


class API_DB:
    PASSWD = "SROS-DB-EXPORT"

    def GET(self):
        TMP_DB_EXPORT_DIR = '/tmp/sros_db_export/'
        TMP_DB_DATA_EXPORT = TMP_DB_EXPORT_DIR + 'sros.db_export'
        clear_path(TMP_DB_EXPORT_DIR)
        os.system('cp /sros/db/main.db3 ' + TMP_DB_EXPORT_DIR)
        os.system("/sros/bin/sros test >> " + TMP_DB_EXPORT_DIR + "version.txt")
        os.system(
            "cd " + TMP_DB_EXPORT_DIR + " && tar czf - * | openssl des3 -md md5 -salt -k " + API_DB.PASSWD + " > " + TMP_DB_DATA_EXPORT)
        return send_file(TMP_DB_DATA_EXPORT)

    def POST(self):
        x = web.input(file={})
        TMP_DB_IMPORT_DIR = '/tmp/sros_db_import/'
        IMPORT_DB_FILE_PATH = TMP_DB_IMPORT_DIR + 'sros.db_import'
        clear_path(TMP_DB_IMPORT_DIR)
        if 'file' in x:
            print("Import db file:", x['file'].filename)

            with open(IMPORT_DB_FILE_PATH, 'wb') as tmp_file:
                tmp_file.write(x['file'].file.read())
                tmp_file.close()

            cmd_str = "cat " + IMPORT_DB_FILE_PATH + " | openssl des3 -md md5 -d -k " + API_DB.PASSWD + " | tar xzvf - -C " + TMP_DB_IMPORT_DIR
            os.system(cmd_str)

            with open(TMP_DB_IMPORT_DIR + "version.txt", 'r') as f:
                old_sros_version = f.readline()
                old_sros_version = old_sros_version[:-1]
            cmd_str = '[[ `/sros/bin/sros test` == "' + old_sros_version + '" ]]'
            print(cmd_str)
            r = os.system(cmd_str)
            current_sros_version = ""
            if r != 0:
                raise web.internalerror("sros version is mismatch! " + old_sros_version + " && " + current_sros_version)

            clear_path("/sros/update/db_import/")
            os.system('cp ' + TMP_DB_IMPORT_DIR + "main.db3 /sros/update/db_import/")
            send_pipe_dat('update_db /sros/update/db_import/main.db3\n')
            return web.OK
        raise web.internalerror("file not exist!")

    # 解决跨域访问问题
    def OPTIONS(self):
        pass


class API_V1_MapList:
    def GET(self):
        try:
            map_name_file_list = get_map_name_list(map_dir=SROS_MAP_DIR)
            map_name_file_display_list = []
            for map_name in map_name_file_list:
                map_name_file_display_list.append(map_name.decode('utf-8'))

            map_name_table_list = []
            map_name_table_list_resp = MapsAdmin.get_all_maps()
            for map_info in map_name_table_list_resp:
                map_name_table_list.append(map_info["name"].decode('utf-8'))

            map_name_file_display_list_set = set(map_name_file_display_list)
            map_name_table_list_set = set(map_name_table_list)

            add_map_name_list = list(map_name_file_display_list_set - map_name_table_list_set)
            up_map_name_list = list(map_name_file_display_list_set & map_name_table_list_set)
            rm_map_name_list = list(map_name_table_list_set - map_name_file_display_list_set)

            for name in rm_map_name_list:
                MapsAdmin.delete_map_by_name(name.decode('utf-8'))

            for name in add_map_name_list:
                map_info = get_map_item_info(map_name=name.encode('utf-8'), map_dir=SROS_MAP_DIR)
                map_info['name'] = map_info['name'].decode('utf-8')
                MapsAdmin.create_map(map_info)

            for name in up_map_name_list:
                map_table_item = MapsAdmin.get_map_info_by_name(name)
                map_info = get_map_item_info(map_name=name.encode('utf-8'), map_dir=SROS_MAP_DIR)

                if map_info['md5'] != map_table_item['md5']:
                    map_info['name'] = map_info['name'].decode('utf-8')
                    MapsAdmin.update_map_by_name(name, map_info)
        except Exception as e:
            raise web.internalerror("Exception error")
        except IOError as e:
            raise web.internalerror("IOError error")
        return json.dumps(MapsAdmin.get_all_maps())

    def POST(self, mode, map_name, val):
        print('update map_list mode,map_name,val', mode, map_name, val)
        map_name = map_name.decode('utf-8')
        if mode == 'top_timestamp':
            return MapsAdmin.update_map_top_timestamp__by_name(map_name, int(val))
        elif mode == 'rename':
            return MapsAdmin.update_map_name_by_name(map_name, val.decode('utf-8'))

    # 解决跨域访问问题
    @staticmethod
    def OPTIONS(mode, map_name, val):
        pass


class API_V2_MapList:
    def GET(self):
        return json.dumps(MapsAdmin.get_all_maps())

    def POST(self):
        try:
            dat = json.loads(web.data())
            os.system('sync')
            map_info = get_map_item_info(map_name=dat.get("map_name").encode('utf-8'), map_dir=SROS_MAP_DIR)
            map_info['name'] = map_info['name'].decode('utf-8')
            map_id = MapsAdmin.create_map(map_info)
            if map_id == None:
                _logger.info("map_id is None ,map_info:%s" % map_info)
                return web.internalerror()
        except Exception as e:
            _logger.error(e)
            raise web.internalerror(e)
        return web.OK

    def DELETE(self, map_name):
        try:
            MapsAdmin.delete_map_by_name(map_name.decode('utf-8'))
        except Exception as e:
            raise web.internalerror(e)
        return web.OK

    # 解决跨域访问问题
    def OPTIONS(self, map_name={}):
        pass


class API_MapList:
    def GET(self):
        map_list = []
        for map_name in get_map_name_list(map_dir=SROS_MAP_DIR):
            map_item = get_map_item_info(map_name=map_name, map_dir=SROS_MAP_DIR)
            map_item["modify_time"] = timestamp_to_timestr(map_item["modify_time"] / 1000)
            map_list.append(map_item)

        data = dict(maps=map_list)
        web.header('Content-Type', 'application/json')
        return json.dumps(data)


class API_MapInfo:
    def GET(self, map_name):
        map_name = map_name.encode('utf-8')
        web.header('Content-Type', 'application/json')
        return json.dumps(get_map_item_meta(map_name))


class API_MapItem:
    # 加载地图json文件
    def GET(self, map_name):
        map_name = map_name.encode('utf-8')
        file_type = web.input(file_type='None').file_type
        print(map_name)
        if map_name not in get_map_name_list(map_dir=SROS_MAP_DIR):
            raise web.NotFound()
        if file_type != 'json' and file_type != 'map' and file_type != 'None':
            raise web.badrequest("unknown file type")
        if file_type == 'None':
            web.header('Content-Type', 'application/json')
            return json.dumps(self.handle_map_json(map_name))
        else:
            return self.handle_map_file(map_name=map_name, file_type=file_type)

    # 这个接口用来干嘛的???导入地图???
    def POST(self, map_name):
        map_name = map_name.encode('utf-8')

        replace = web.input(replace="false")

        # 如果设置了replace=true，那么就不检查地图名冲突
        if replace != "true" and map_name in get_map_name_list(map_dir=SROS_MAP_DIR):
            raise web.Conflict()

        try:
            x = web.input(file={})
        except ValueError:
            raise web.NotAcceptable("File is too large (>100MB).")

        tmp_file_path = '/tmp/web_upload_map.tmp'
        if 'file' in x:
            with open(tmp_file_path, 'wb') as tmp_file:
                tmp_file.write(x['file'].file.read())
                tmp_file.close()

            os.system("tar xvf %s -C /" % tmp_file_path)

        return self.GET(map_name)

    # 删除指定地图
    def DELETE(self, map_name):
        map_name = map_name.encode('utf-8')
        print(map_name)

        if map_name not in get_map_name_list(map_dir=SROS_MAP_DIR):
            raise web.NotFound()

        file_names = generate_map_export(SROS_MAP_DIR, map_name)

        cmd_str = "rm %s " % file_names
        os.system(cmd_str)
        # todo 删除文件同时 删除数据库里面的数据
        try:
            MapsAdmin.delete_map_by_name(map_name)
        except Exception as e:
            raise web.internalerror("delete %s sql data is failed" % file_names)
        return json.dumps({})

    def handle_map_json(self, map_name):
        if map_name not in get_map_name_list(map_dir=SROS_MAP_DIR):
            raise web.NotFound()
        data = get_map_item_json(map_name=map_name, map_dir=SROS_MAP_DIR)
        if data is None:
            raise web.NotFound()
        return data

    def handle_map_file(self, map_name, file_type):
        map_file_path = get_map_file_path(map_name=map_name, file_type=file_type, map_dir=SROS_MAP_DIR)
        if map_file_path == "":
            raise web.NotFound()
        return send_file(map_file_path)


class API_MAPItemUpdate:
    # 传入地图json文件，保存json文件，同时生成.map文件
    def POST(self, map_name):
        map_name = map_name.encode('utf-8')
        req_str = web.data().encode('utf-8')
        response = {"result": 0}
        try:
            json_dat = json.loads(req_str)
        except ValueError:
            raise web.BadRequest("json format error")

        if update_map(map_name, json_dat):
            try:
                map_table_item = MapsAdmin.get_map_info_by_name(map_name)
                map_info = get_map_item_info(map_name=map_name.encode('utf-8'), map_dir=SROS_MAP_DIR)

                if map_info['md5'] != map_table_item['md5']:
                    map_info['name'] = map_info['name'].decode('utf-8')
                    MapsAdmin.update_map_by_name(map_name, map_info)
            except Exception as e:
                raise web.internalerror(e)
            response["result"] = 1

        return json.dumps(response)

    @staticmethod
    # 解决跨域访问问题
    def OPTIONS(self):
        pass


class API_MapCopyPath:
    # 从地图A拷贝路网到地图B
    def POST(self, from_map, to_map):
        print("copy from " + from_map + " to " + to_map)
        map_from = from_map.encode('utf-8')
        map_to = to_map.encode('utf-8')
        if map_from == "" or map_to == "":
            raise web.BadRequest("invalid input map name")
        if self.copy_map_road_path(map_from, map_to):
            try:
                map_table_item = MapsAdmin.get_map_info_by_name(to_map)
                map_info = get_map_item_info(map_name=to_map.encode('utf-8'), map_dir=SROS_MAP_DIR)

                if map_info['md5'] != map_table_item['md5']:
                    map_info['name'] = map_info['name'].decode('utf-8')
                    MapsAdmin.update_map_by_name(to_map, map_info)
            except Exception as e:
                raise web.internalerror(e)
            response = {"result": 1}
        else:
            response = {"result": 0}
        return json.dumps(response)

    def copy_map_road_path(self, from_map, to_map):
        print("copy map json from " + from_map + " to " + to_map)
        if from_map == "" or to_map == "":
            return False
        map_from_json_file_path = get_map_file_path(from_map, "json")
        map_to_json_file_path = get_map_file_path(to_map, "json")
        map_from_json = json.loads(read_file(map_from_json_file_path))
        map_to_json = json.loads(read_file(map_to_json_file_path))
        if "data" not in map_from_json or "data" not in map_to_json:
            raise web.internalerror("Invalid map json format: data not find")
        map_to_json["data"] = map_from_json["data"]
        if "private" in map_from_json:
            map_to_json["private"] = map_from_json["private"]
        else:
            map_to_json["private"] = {}
        return update_map(to_map, map_to_json)

    # 解决跨域访问问题
    @staticmethod
    def OPTIONS(from_map, to_map):
        pass


class API_MapItemImage:
    def GET(self, map_name):
        map_name = map_name.encode('utf-8')
        scale = web.input(scale="").scale
        if scale == "" or int(scale) == 1:
            now_image_path = "/sros/map/%s.png" % map_name
        else:
            now_image_path = "/sros/map/%s_%s.png" % (map_name, scale)

        now_image_path = now_image_path.encode('utf-8')

        print("get map image file: %s" % now_image_path)

        web.header("Content-Type", "images/png")
        if map_name not in get_map_name_list(map_dir=SROS_MAP_DIR):
            raise web.NotFound()

        if not os.path.exists(now_image_path) or os.path.getsize(now_image_path) == 0:
            print(map_name, "has no png image")
            create_map_image(map_name)
        if not os.path.exists(now_image_path):
            raise web.NotFound()
        return open(now_image_path, "rb").read()


class API_MapExport:
    def GET(self, name):
        name = name.encode('utf-8')
        export_type = web.input(type="FMS").type
        has_packed = web.input(packed=0).packed
        if export_type != 'FMS' and export_type != 'OM':
            raise web.BadRequest('unkown type！')

        # combine and compress map
        if export_type == "OM":
            export_file_name = "/tmp/%s_opt.map_export" % name
        else:
            export_file_name = "/tmp/%s_nav.map_export" % name

        # 可能已经存在，但是是很久以前打包的老文件
        if has_packed and os.path.exists(export_file_name):
            print('Map files file has packed')
        elif not self.pack_export_map(name, export_type):
            return web.InternalError("Failed to package map files")
        return send_file(export_file_name)

    # 由于导出地图包时，打包压缩话费时间太长了，因此支持先打包后再请求文件
    def POST(self, map_name):
        map_name = map_name.encode('utf-8')
        dat = json.loads(web.data())
        export_type = dat['type']
        if self.pack_export_map(map_name, export_type):
            return web.OK
        else:
            return web.InternalError('Failed to pack map files')

    # 解决跨域访问问题
    def OPTIONS(self, map_name):
        pass

    def pack_export_map(self, map_name, export_type):
        if map_name not in get_map_name_list(map_dir=SROS_MAP_DIR):
            raise web.NotFound('map not exist!')
            return False

        file_names = generate_map_export(SROS_MAP_DIR, map_name, export_type)
        assert file_names != "", "不存在找不到一个文件的情况！"

        # combine and compress map
        if export_type == "OM":
            export_file_name = "/tmp/%s_opt.map_export" % map_name
        else:
            export_file_name = "/tmp/%s_nav.map_export" % map_name
        cmd_str = "tar cvzf %s %s" % (export_file_name, file_names)
        r = os.system(cmd_str)
        if r != 0:
            raise web.internalerror("export failed (%d)" % r)
        os.system('sync')
        return True


class API_MapImport:
    def POST(self):
        # TODO: 封装成函数
        try:
            x = web.input(file={})
            client_md5 = str(web.input(md5='').md5)
        except ValueError:
            _logger.error("File is too large (>100MB).")
            raise web.NotAcceptable("File is too large (>100MB).")

        IMPORT_MAP_FILE_PATH = "/tmp/import-map.tar.bz"
        if 'file' in x:
            with open(IMPORT_MAP_FILE_PATH, 'wb') as tmp_file:
                tmp_file.write(x['file'].file.read())
                tmp_file.close()

            if not check_file_md5(IMPORT_MAP_FILE_PATH, client_md5):
                _logger.error('file md5 check error')
                raise web.BadRequest('file md5 check error')
            result = os.popen('tar xvf ' + IMPORT_MAP_FILE_PATH + '  -C /')
            os.system('sync')
            map_name = ''
            while True:
                line = result.readline().strip()
                if not line:
                    break
                if os.path.splitext(line)[-1] == '.json':
                    basename = os.path.basename(line)
                    map_name = os.path.splitext(basename)[0]
                    break
            result.close()

            if map_name != '':
                create_map_json(map_name)
                create_map_image(map_name)
                os.system('sync')
                send_pipe_dat('update_map ' + map_name + '\n')
                _logger.info("%s write to pipe for update map" % map_name)

                try:
                    map_info = get_map_item_info(map_name=map_name.encode('utf-8'), map_dir=SROS_MAP_DIR)
                    map_info['name'] = map_info['name'].decode('utf-8')
                    if not MapsAdmin.has_map(map_info["name"]):
                        map_id = MapsAdmin.create_map(map_info)
                    else:
                        MapsAdmin.update_map_by_name(map_info["name"], map_info)
                except Exception as e:
                    _logger.error(e)
                    raise web.internalerror(e)

        return web.OK

    def OPTIONS(self):
        pass


class API_TaskExport:
    def GET(self, task_id):
        print("Export task: " + task_id)
        if int(task_id) <= 0:
            raise web.BadRequest("Invalid task id")
        task = UserMissionAdmin.get_detailed_mission(int(task_id))
        if not task:
            raise web.notfound()

        export_path = get_export_task_file_path()
        clear_path(export_path)

        try:
            task_name = task["name"].encode('utf-8')
            create_task_json_file(task)
            # 压缩日志
            dt = datetime.now()
            export_task_file_path = "/tmp/" + task_name + "-" + dt.strftime('%Y-%m-%d-%H-%M-%S') + ".task_export"

            record_dir = os.getcwd()
            cmd_str = 'tar cvzf %s *' % export_task_file_path
            os.chdir(export_path)
            print(cmd_str)
            os.system(cmd_str)
            os.chdir(record_dir)
            os.system('sync')
            return send_file(export_task_file_path)
        except IndexError:
            raise web.internalerror("Index error")
        except Exception:
            raise web.internalerror()
        # finally: # 删除会导致文件还没发送就删除了
        #     os.remove(file_path)

    def OPTIONS(self, task_id):
        pass


class API_TaskImport:
    def POST(self):
        # TODO: 封装成函数
        print("Import task")
        try:
            x = web.input(file={})
            client_md5 = str(web.input(md5='').md5)
            is_override = int(web.input(override='0').override)
        except ValueError:
            raise web.NotAcceptable("Import task file error")

        file_path = "/tmp/import_task.tar.gz"
        if 'file' in x:
            print("Import task file:", x['file'].filename)

            with open(file_path, 'wb') as tmp_file:
                tmp_file.write(x['file'].file.read())
                tmp_file.close()
            os.system('sync')
            if not check_file_md5(file_path, client_md5):
                raise web.BadRequest('file md5 check error')

            extract_path = get_import_task_file_path()
            clear_path(extract_path)

            cmd_str = 'tar zxvf %s -C %s' % (file_path, extract_path)
            print(cmd_str)
            os.system(cmd_str)
            os.system('sync')
            try:
                # TODO 注意：这里没有更新owner_id,但是普通用户不能导入任务，
                task_files = get_files_from_path(extract_path, ".json")
                # print(task_files)
                map_task_id = dict()  # 记录任务原来id和导入后任务id,用于在导入任务后更新与子任务的绑定
                for file_path in task_files:
                    file_content = read_file(file_path)
                    task_json = json.loads(file_content)
                    task_json["mapName"] = task_json["map_name"]
                    old_task_id = task_json["id"]
                    del task_json["map_name"]
                    del task_json["id"]  # 删除id由id自增
                    if "top_timestamp" not in task_json:
                        task_json['top_timestamp'] = 0

                    exist_mission = UserMissionAdmin.get_detailed_mission_by_name(task_json["name"],
                                                                                  task_json["mapName"])
                    if not exist_mission:
                        task_json["body"] = json.dumps(task_json["body"])
                        new_task_id = create_mission(task_json)
                    elif is_override:
                        new_task_id = exist_mission["id"]
                        task_json["id"] = new_task_id  # 导入任务时，覆盖已经存在的任务
                        UserMissionAdmin.update_detailed_mission(task_json)
                        if exist_mission["item_valid"] == 0:
                            UserMissionAdmin.restore_mission(task_json["id"])
                    map_task_id[old_task_id] = new_task_id

                # 导入任务后更新任务与子任务之间的绑定
                update_subtask_id(map_task_id)
            except Exception as e:
                print(e)
                raise web.badrequest("Task file format error")
            return web.OK
        else:
            return web.BadRequest("No file content")

    def OPTIONS(self):
        pass


class API_TaskTemplate:
    def GET(self):
        tpl_id = int(web.input(id='0').id)
        if tpl_id > 0:
            tpl_dat = TaskTemplateAdmin.get_template(tpl_id)
        else:
            tpl_dat = TaskTemplateAdmin.get_all_templates()
        web.header('Content-Type', 'application/json')
        return json.dumps(tpl_dat)

    def POST(self):
        print("create task template")
        tpl_dat = json.loads(web.data())
        tpl_dat['body'] = json.dumps(tpl_dat['body'])
        return TaskTemplateAdmin.create_template(tpl_dat)

    def PUT(self):
        print("update task template")
        tpl_dat = json.loads(web.data())
        tpl_dat['body'] = json.dumps(tpl_dat['body'])
        TaskTemplateAdmin.update_template(tpl_dat)
        return web.OK

    def DELETE(self, tpl_id):
        tpl_id = int(tpl_id)
        print("delete task template " + str(tpl_id))
        if tpl_id <= 0:
            raise web.badrequest("Invalid input params")
        TaskTemplateAdmin.delete_template(tpl_id)

    def OPTIONS(self, tpl_id=0):
        pass


class API_TaskTemplateImport:
    def POST(self):
        # TODO: 封装成函数
        print("Import task template")
        try:
            x = web.input(file={})
            client_md5 = str(web.input(md5='').md5)
            is_override = int(web.input(override='0').override)
        except ValueError:
            raise web.NotAcceptable("Import task file error")

        file_path = "/tmp/import_tpl.tar.gz"
        if 'file' in x:
            print("Import task template file:", x['file'].filename)

            with open(file_path, 'wb') as tmp_file:
                tmp_file.write(x['file'].file.read())
                tmp_file.close()
            os.system('sync')
            if not check_file_md5(file_path, client_md5):
                raise web.BadRequest('file md5 check error')

            extract_path = get_import_task_file_path()
            clear_path(extract_path)

            cmd_str = 'tar zxvf %s -C %s' % (file_path, extract_path)
            print(cmd_str)
            os.system(cmd_str)
            os.system('sync')

            try:
                tpl_files = get_files_from_path(extract_path, ".json")
                # print(tpl_files)
                map_tpl_id = dict()
                for tpl_file in tpl_files:
                    file_content = read_file(tpl_file)
                    task_json = json.loads(file_content)
                    task_json["body"] = json.dumps(task_json["body"])
                    old_tpl_id = task_json["id"]
                    del task_json["id"]  # 删除id由id自增

                    exist_tpl = TaskTemplateAdmin.get_template_by_name(task_json["name"])
                    if exist_tpl is not None:
                        # 内置模板不允许覆盖
                        if TaskTemplateAdmin.is_system_template(task_json["name"]):
                            return web.Forbidden("It is not allow to override system template")
                        if is_override:
                            new_tpl_id = exist_tpl["id"]
                            task_json["id"] = new_tpl_id  # 根据id更新
                            TaskTemplateAdmin.update_template(task_json)
                    else:
                        new_tpl_id = TaskTemplateAdmin.create_template(task_json)
                    map_tpl_id[old_tpl_id] = new_tpl_id

                # 更新与子任务绑定关系
                update_subtask_id(map_tpl_id, True)
            except Exception as e:
                print(e)
                raise web.badrequest("Task file format error")
            return web.OK
        else:
            return web.BadRequest("No file content")

    def OPTIONS(self):
        pass


class API_TaskTemplateExport:
    def GET(self, tpl_id):
        print("Export task template: " + tpl_id)
        if int(tpl_id) <= 0:
            raise web.BadRequest("Invalid task id")
        template = TaskTemplateAdmin.get_template(int(tpl_id))
        if not template:
            raise web.notfound()

        export_path = get_export_task_file_path()
        clear_path(export_path)

        try:
            tpl_name = template["name"].encode('utf-8')
            template["body"] = json.dumps(template["body"])
            create_task_json_file(template, True)

            # 压缩日志
            dt = datetime.now()
            export_task_file_path = "/tmp/" + tpl_name + "-" + dt.strftime('%Y-%m-%d-%H-%M-%S') + ".tpl_export"

            record_dir = os.getcwd()
            cmd_str = 'tar cvzf %s *' % export_task_file_path
            os.chdir(export_path)
            print(cmd_str)
            os.system(cmd_str)
            os.chdir(record_dir)
            os.system('sync')
            return send_file(export_task_file_path)
        except IndexError:
            raise web.internalerror("Index error")
        except Exception:
            raise web.internalerror()
        # finally: # 删除会导致文件还没发送就删除了
        #     os.remove(file_path)

    def OPTIONS(self, task_id):
        pass


class API_SettingList:
    def GET(self):
        ds = get_settings_list()
        web.header('Content-Type', 'application/json')
        return json.dumps(dict(settings=ds))


class API_SettingItem:
    def GET(self, id):
        rs = db.select('config', where="id = %d" % int(id))

        try:
            web.header('Content-Type', 'application/json')
            r = rs[0]
            r['changed_time'] = timestamp_to_timestr(r['changed_time'])
            return json.dumps(r)
        except IndexError:
            raise web.notfound()
        except Exception:
            raise web.internalerror()

    def PATCH(self, id):
        input = web.data()

        data = json.loads(input)

        print(data["value"])

        r = db.update("config", where="id = %s" % id,
                      value=data["value"],
                      changed_time=int(time.time())
                      )

        if r == 0:
            raise web.notfound()
        else:
            return self.GET(id)


class MapList:
    def GET(self):
        map_list = []
        for map_name in get_map_name_list(map_dir=SROS_MAP_DIR):
            map_item = get_map_item_info(map_name=map_name, map_dir=SROS_MAP_DIR)
            map_item["modify_time"] = timestamp_to_timestr(map_item["modify_time"] / 1000)
            map_list.append(map_item)

        print(map_list)
        return render.map_list('map_list', map_list)


class API_CameraNow:
    def GET(self, camera_id):
        web.header("Content-Type", "images/jpeg")

        now_image_path = "/tmp/camera_%s_now.jpg" % camera_id

        if not os.path.exists(now_image_path):
            return web.notfound()

        return open(now_image_path, "rb").read()


class API_VisionNow:
    def GET(self, vision_id):
        web.header("Content-Type", "images/jpeg")

        now_image_path = "/tmp/vision_%s_now.jpg" % vision_id

        if not os.path.exists(now_image_path):
            return web.notfound()

        return open(now_image_path, "rb").read()


class API_Mission:
    def GET(self, mission_id):
        mission = UserMissionAdmin.get_detailed_mission(int(mission_id))
        # print(mission)
        if not mission:
            raise web.notfound()
            return
        web.header('Content-Type', 'application/json')
        mission['body'] = json.loads(mission['body'])
        return json.dumps(mission)

    def POST(self, mission_id):
        print("create mission")
        try:
            mission = json.loads(web.data())
            mission['body'] = json.dumps(mission['body'])
            return create_mission(mission)
        except Exception as e:
            print(e)
            raise web.internalerror()

    def PUT(self, mission_id):
        print("update mission")
        try:
            mission = json.loads(web.data())
            # print(mission)
            UserMissionAdmin.update_detailed_mission(mission)
        except Exception as e:
            print(e)
            raise web.internalerror()

    def DELETE(self, mission_id):
        mission_id = int(mission_id)
        try:
            UserMissionAdmin.delete_mission(mission_id)
        except Exception as e:
            print(e)
            raise web.internalerror()

    def OPTIONS(self, mission_id):
        pass


class API_MissionList:
    def GET(self, user_id):
        user_id = int(user_id)
        mission_list = UserMissionAdmin.get_missions(user_id)
        web.header('Content-Type', 'application/json')
        return json.dumps(mission_list)


class API_ClassicLogExport:
    def GET(self):
        # 压缩日志，重写NetworkModule::classicLogExport函数
        file_names_str = ''
        file_too_old_str = ''  # 记录老的日志
        is_debug = web.input(debug=False).debug
        for dirpath, dirnames, filenames in os.walk('/sros/log/'):
            for file_name in filenames:
                if 'sros.' in file_name:
                    file_path = os.path.join(dirpath, file_name)
                    if os.path.islink(file_path):  # 软链接文件如sros.FATAL
                        file_names_str += " " + file_path  # 记录需要打包的日志
                        continue
                    export_time_s = 3 * 24 * 60 * 60  # 三天的日志
                    if is_debug:
                        export_time_s = 10 * 24 * 60 * 60  # 十天的日志
                    if time.time() - os.stat(file_path).st_mtime > export_time_s:  # 超过两天的情况
                        file_too_old_str += " " + file_path
                    else:
                        file_names_str += " " + file_path  # 记录需要打包的日志

        # print(file_names_str)
        # 删除超过两天的日志
        if len(file_names_str) == 0 and len(file_too_old_str) != 0:
            file_names_str = file_too_old_str

        nickname = ''  # 车辆的别名
        # 生成配置文件
        rs = db.select('config', where="is_valid=1")
        with open('/sros/log/cfg.ini', 'w') as f:
            for r in rs:
                try:
                    if r.key == 'main.nickname':
                        nickname = r.value
                        nickname = nickname.replace("/", "-")  # 斜杠会导致日志导出失败，所以替换成短杠
                        nickname = nickname.replace("&", "-")  # $会导致日志导出失败，所以替换成短杠
                        nickname = nickname.replace(" ", "-")  # 会导致日志导出失败，所以替换成短杠
                    line = r.key + ',' + r.name + ',' + r.value + '\n'
                    f.write(line)
                except:
                    print('error config! id is ' + r.idz)

        # 获取其他日志
        CONFIG_FILE = "/sros/log/cfg.ini"
        STM32_UPDATE_LOG = '/sros/stm32_update.log'
        SROS_UPDATE_LOG = "/sros/update.log"
        if os.path.exists(CONFIG_FILE):
            file_names_str += " " + CONFIG_FILE
        if os.path.exists(STM32_UPDATE_LOG):
            file_names_str += " " + STM32_UPDATE_LOG
        if os.path.exists(SROS_UPDATE_LOG):
            file_names_str += " " + SROS_UPDATE_LOG

        # 获取journalctl中的日志，有时候时间不对，导致有些关键的日志没有能导出
        if is_debug:
            SROS_JOURNALS_LOG = "/sros/log/system.journals"
            cmd_str = "journalctl | grep -vE 'Got frame errors|Got Break' > " + SROS_JOURNALS_LOG
            for dirpath, dirnames, filenames in os.walk('/sros/log/'):
                if dirpath == '/sros/log/':
                    for dir_name in dirnames:
                        file_names_str += " /sros/log/" + dir_name
                    break
        else:
            SROS_JOURNALS_LOG = "/sros/log/sros.journals"
            cmd_str = 'journalctl -u sros -u ui_server -n 20000 > ' + SROS_JOURNALS_LOG
        os.system(cmd_str)
        file_names_str += " " + SROS_JOURNALS_LOG

        # 压缩日志
        dt = datetime.now()
        EXPORT_LOG_FILE_PATH = "/tmp/" + nickname + "-sros-" + dt.strftime('%Y-%m-%d-%H-%M-%S') + ".log_export"
        print("EXPORT_LOG_FILE_PATH: " + EXPORT_LOG_FILE_PATH)
        cmd_str = 'tar cvzf - ' + file_names_str + ' > ' + EXPORT_LOG_FILE_PATH
        print(cmd_str)
        os.system(cmd_str)
        return send_file(EXPORT_LOG_FILE_PATH)


class API_RunLogExport:
    def GET(self):
        offset = web.input(offset=0).offset  # 起始位置
        offset = int(offset)
        offset = max(0, offset)
        limit = web.input(limit=10).limit  # 一次读取多少条
        limit = int(limit)
        limit = max(1, min(limit, 100))
        order = web.input(order="DESC").order
        if order != "desc" and order != "asc" and order != "DESC" and order != "ASC":
            raise web.badrequest("Invalid input params, order must be desc or asc")
        order = "id " + order

        rs = db.select('run_log', what="id, start_time, last_alive_time, move_mileage", offset=offset, limit=limit,
                       order=order)
        data = []
        for r in rs:
            data.append(dict(r))
        web.header('Content-Type', 'application/json')
        return json.dumps(data)


# NOTE: 先改logger.h中的模块名然后来改此处，两处需要同步
UserLogModules = {
    "command-hander": True,
    "protobuf": False,
    "modbus": False,
    "movement-task": True,
    "action-task": True,
    "exec-error": True,
    "sros": True,
    "task": True,
    "device": True
}


class API_ClassicUserLogFollow:
    def GET(self):
        lines = web.input(lines=20).lines  # 获取多少行，取值范围[1, 1000]
        lines = int(lines)
        lines = max(1, min(lines, 1000))
        modules = copy.deepcopy(UserLogModules)
        for key in modules:
            d = dict()
            d[key] = modules[key]
            value = str(web.input(**d)[key])
            if value == 'true' or value == '1':
                modules[key] = True
            elif value == 'false' or value == '0':
                modules[key] = False
        print(modules)
        level = web.input(level="info").level
        if level != "fatal" and level != "error" and level != "warning" and level != "info":
            level = "info"

        if os.system('ls /sros/log/*.json') != 0:  # 先测试是否存在*.json文件，若不存在导出不会成功
            raise web.NotFound("not log file created!")

        invert_match_str = ""  # 不匹配的内容
        for key in modules:
            if modules[key] is False:
                invert_match_str += '"key":"' + key + '"|'

        if level == "fatal":
            invert_match_str += '"severity":"error"|"severity":"warning"|"severity":"info"|'
        elif level == "error":
            invert_match_str += '"severity":"warning"|"severity":"info"|'
        elif level == "warning":
            invert_match_str += '"severity":"info"|'

        invert_match_str = invert_match_str[:-1]  # 删除掉多余的‘|’，而且上面的需要保证最后一个为‘|’

        if invert_match_str == "":
            cmd_str = "grep -h '' `ls -tr /sros/log/*.json | tail -n 2` | tail -n " + str(lines)
        else:
            cmd_str = "grep -hvE '" + invert_match_str + "' `ls -tr /sros/log/*.json | tail -n 2` | tail -n " + str(
                lines)
        print(cmd_str)
        stdin, stdout = os.popen2(cmd_str)
        stdin.close()
        lines = stdout.readlines()
        stdout.close()

        logs = []
        for line in lines:
            try:
                logs.append(json.loads(line))
            except:
                print(line)

        data = {
            "modules": modules,
            "logs": logs
        }

        web.header('Content-Type', 'application/json')
        return json.dumps(data)


class API_ClassicUserLogSection:
    def GET(self):
        begin = web.input(begin=0).begin  # 从多少行开始获取
        lines = web.input(lines=20).lines  # 获取多少行，取值范围[1, 1000]
        begin = int(begin)
        lines = int(lines)
        lines -= 1
        lines = max(1, min(lines, 1000))
        begin = max(1, begin)
        modules = copy.deepcopy(UserLogModules)

        # 过滤
        for key in modules:
            d = dict()
            d[key] = modules[key];
            value = str(web.input(**d)[key])
            if value == 'true' or value == '1':
                modules[key] = True
            elif value == 'false' or value == '0':
                modules[key] = False
        print(modules)
        level = web.input(level="info").level
        if level != "fatal" and level != "error" and level != "warning" and level != "info":
            level = "info"

        if os.system('ls /sros/log/*.json') != 0:  # 先测试是否存在*.json文件，若不存在导出不会成功
            raise web.NotFound("not log file created!")

        invert_match_str = ""  # 不匹配的内容
        for key in modules:
            if modules[key] is False:
                invert_match_str += '"key":"' + key + '"|'

        if level == "fatal":
            invert_match_str += '"severity":"error"|"severity":"warning"|"severity":"info"|'
        elif level == "error":
            invert_match_str += '"severity":"warning"|"severity":"info"|'
        elif level == "warning":
            invert_match_str += '"severity":"info"|'

        invert_match_str = invert_match_str[:-1]  # 删除掉多余的‘|’，而且上面的需要保证最后一个为‘|’

        # FIXME(pengjiali): 用awk解决获取到到多少条，然后由于过滤在后面导致过滤后请求的条数和给出的条数不对等的问题，grep暂时无法解决这个问题。
        cmd_str = "grep -h '\"id\":" + str(begin) + ",' -A " + str(lines) + " /sros/log/*.json"
        if invert_match_str == "":
            cmd_str = "grep -h '\"id\":" + str(begin) + ",' -A " + str(lines) + " /sros/log/*.json"
        else:
            cmd_str = "grep -h '\"id\":" + str(
                begin) + ",' -A 20000 /sros/log/*.json | grep -vE '" + invert_match_str + "' | head -n " + str(lines)
        print(cmd_str)

        stdin, stdout = os.popen2(cmd_str)
        stdin.close()
        lines = stdout.readlines()
        stdout.close()
        logs = []
        for line in lines:
            try:
                logs.append(json.loads(line))
            except:
                print(line)
        web.header('Content-Type', 'application/json')
        return json.dumps(logs)


class API_SystemUpdateImport:
    def POST(self):
        try:
            x = web.input(file={})
            client_md5 = str(web.input(md5='').md5)
        except ValueError:
            raise web.NotAcceptable("File is too large (>100MB).")

        tmp_file_path = '/tmp/web_upload_upgrade_package.tmp'
        if 'file' in x:
            with open(tmp_file_path, 'wb') as tmp_file:
                tmp_file.write(x['file'].file.read())
                tmp_file.close()

            if not check_file_md5(tmp_file_path, client_md5):
                raise web.BadRequest('file md5 check error')

            IMPORT_UPDATE_FILE_PATH = "/sros/update/sros-import-update.tar.bz"
            os.system("cp %s %s" % (tmp_file_path, IMPORT_UPDATE_FILE_PATH))
            # os.system('systemctl restart sros')

            send_pipe_dat('update_sros ' + IMPORT_UPDATE_FILE_PATH + '\n')
            print("sros update")
            # 初始化地图表的数据
            try:
                generate_maps_db_data()
            except Exception as e:
                raise e
            return web.OK
        return web.internalerror()

    def OPTIONS(self):
        pass


class API_File:
    def GET(self):
        src_file = web.input().get("file")
        if not src_file:
            raise web.BadRequest("Invalid source file")
        return send_file(src_file)


class API_SrcUpdateImport:
    def POST(self):
        try:
            x = web.input(file={})
            client_md5 = str(web.input(md5='').md5)
        except ValueError:
            raise web.NotAcceptable("File is too large (>1MB).")

        tmp_file_path = '/tmp/web_upload_upgrade_package.tmp'
        tmp_real_src_update_name = '/tmp/real_src_update_name.tmp'  # 记录解包后真实的src文件名
        if 'file' in x:
            with open(tmp_file_path, 'wb') as tmp_file:
                tmp_file.write(x['file'].file.read())
                tmp_file.close()

        if not check_file_md5(tmp_file_path, client_md5):
            raise web.BadRequest('file md5 check error')

        src_update_file_path = '/sros/update/' + x['file'].filename
        print(src_update_file_path)
        if src_update_file_path.endswith("src_update"):
            src_update_file_path = src_update_file_path[:-10] + 'bin'  # 传进来的文件是以".src_update"结尾, 解压后以bin结尾
            SRC_UPDATE_KEY = "SRC-666-update";
            cmd_str = "cat " + tmp_file_path + " | openssl des3 -md md5 -d -k " + SRC_UPDATE_KEY + " | tar xzvf - -C /sros/update > " + tmp_real_src_update_name
            ret = os.system(cmd_str)
            if ret != 0:
                raise web.BadRequest('decompress failed!')
            # 处理可能传入的压缩包被改过名字的问题
            real_src_update_name = open(tmp_real_src_update_name, 'r').read()
            if not (real_src_update_name.startswith('src_v') or real_src_update_name.startswith('srtos_v')):
                raise web.BadRequest('The src update package was illegally modified!')
            real_src_update_name = real_src_update_name[:-1]  # 去掉读取一行末尾的换行符
            src_update_file_path = '/sros/update/' + real_src_update_name
        elif src_update_file_path.endswith("bin"):
            os.system("cp '%s' '%s'" % (tmp_file_path, src_update_file_path))
        else:
            raise web.BadRequest('Only accept .bin and .src_update suffix file!')
        # 模拟生成md5校验
        cmd_str = "md5sum '" + src_update_file_path + "' > '" + src_update_file_path + "_md5sum.txt'"
        os.system(cmd_str)

        send_pipe_dat('update_src ' + src_update_file_path + '\n')
        print("src update")
        return web.OK

    def OPTIONS(self):
        pass


class API_Performance:
    def GET(self):
        # monitor文件名称中数字对应时间戳单位是s
        from_time = int(web.input(fromTime=0).fromTime)
        to_time = int(web.input(toTime=int(round(time.time() * 1000))).toTime)
        # 一个json文件大概5M考虑内存限制，暂时最多只允许访问5天内数据
        if (to_time - from_time) > 5 * 24 * 3600 * 1000:
            print("Time range is larger than 5 days")
            raise web.BadRequest

        # 参数格式: systemHardware.cpuUsage,systemHardware.memoryUsage
        # systemHardware表示所属二级模块，cpuUsage表示模块中的字段名
        dat_type = web.input(type='').type
        if dat_type == "":
            print("Invalid input monitor parameter")
            raise web.BadRequest
        items = dat_type.split(",")
        block_dat = monitor_parser.parse_from_to(int(from_time / 1000), int(to_time / 1000), items)
        web.header('Content-Type', 'application/json')
        return json.dumps({'dat': block_dat})


class API_LMK:
    def GET(self):
        map_name = web.input(map="").map
        map_name = map_name.encode('utf-8')
        if map_name == "":
            raise web.BadRequest("Invalid input map name")
        lmk_list = LMKParser.get_lmk_list(map_name)
        web.header('Content-Type', 'application/json')
        return json.dumps(lmk_list)


class API_QRCode:
    def GET(self, map_name):
        map_name = map_name.encode("utf-8")
        if map_name is None:
            raise web.BadRequest("Invalid input map name")
        return json.dumps(QRCodeParser.get_qrcode_list(map_name))

    def POST(self):
        print("update fuseLocateQRCode")
        # resp = {"state": True, "code": 9999, "message": "ok", "data": {}}
        try:
            # dm_config = ConfigAdmin.get_config('slam.enable_dm_code_location')
            # if dm_config['is_valid'] == 0 or dm_config['value'] == 'False':
            #     return json.dumps(resp)
            qrcode_front = json.loads(web.data())
            if QRCodeParser.set_qrcode_list(qrcode_front):
                return json.dumps({"state": True, "code": 9999, "message": "ok", "data": {}})
        except Exception as e:
            print(e)
            return json.dumps({"state": False, "code": 9999, "message": "fail", "data": {}})

    def OPTIONS(self, schedule_id=0):
        pass


class API_Schedule:
    def GET(self):
        schedules = ScheduleAdmin.get_all_schedules()
        for schedule in schedules:
            schedule["isEnable"] = schedule["is_enable"]
            schedule["taskId"] = schedule["task_id"]
            schedule["desc"] = schedule["detail"]
            schedule["dat"] = json.loads(schedule["dat"])
            del schedule["detail"]
            del schedule["is_enable"]
            del schedule["task_id"]
        web.header('Content-Type', 'application/json')
        return json.dumps(schedules)

    def POST(self):
        print("create schedule")
        try:
            schedule_task = json.loads(web.data())
            result = ScheduleAdmin.create(schedule_task)
            send_pipe_dat('update_schedule ' + str(result) + '\n')
            return result
        except Exception as e:
            print(e)
            raise web.internalerror()

    def PUT(self, schedule_id):
        print("update schedule: " + str(schedule_id))
        try:
            schedule_task = json.loads(web.data())
            # print(mission)
            if ScheduleAdmin.update(schedule_task):
                send_pipe_dat('update_schedule ' + str(schedule_id) + '\n')
                return web.ok
            else:
                raise web.internalerror()
        except Exception as e:
            print(e)
            raise web.internalerror()

    def DELETE(self, schedule_id):
        schedule_id = int(schedule_id)
        print("delete schedule " + str(schedule_id))
        try:
            if ScheduleAdmin.delete(schedule_id):
                send_pipe_dat('update_schedule ' + str(schedule_id) + '\n')
                return web.ok
            else:
                raise web.internalerror()
        except Exception as e:
            print(e)
            raise web.internalerror()

    def OPTIONS(self, schedule_id=0):
        pass


class API_FixedRecord:
    def GET(self):
        try:
            fixed_records = FixedRecord.get_all_records()
            for record in fixed_records:
                record["robotSN"] = record["robot_sn"]
                record["powerCycle"] = record["poweron_number"]
                del record["robot_sn"]
                del record["poweron_number"]
        except BaseException as e:
            raise web.InternalError(e.message)
        web.header('Content-Type', 'application/json')
        return json.dumps(fixed_records)

    def POST(self):
        print("create fixed record")
        try:
            fixed_record = json.loads(web.data())
            result = FixedRecord.create(fixed_record)
            return result
        except Exception as e:
            print(e)
            raise web.internalerror(e.message)

    def OPTIONS(self):
        pass


class API_SrosUpgradeRecord:
    def GET(self):
        try:
            records = UpgradeRecord.get_all_records(0x1)
            web.header('Content-Type', 'application/json')
            return json.dumps(records)
        except Exception as e:
            print(e)
            raise web.internalerror(e.message)


class API_SrcUpgradeRecord:
    def GET(self):
        try:
            records = UpgradeRecord.get_all_records(0x2)
            web.header('Content-Type', 'application/json')
            return json.dumps(records)
        except Exception as e:
            print(e)
            raise web.internalerror(e.message)


class API_MissionRecordClear:
    def GET(self):
        try:
            return json.dumps(
                {"state": True, "code": 9999, "message": "ok", "data": {"count": MissionRecord.set_all_clear()}})
        except Exception as e:
            print(e)
            raise web.internalerror(e.message)


class API_ConfigInfoExport:
    def GET(self, id_str):
        print("Export configs : " + id_str)
        # eg: id_str = 101_601_1813 101
        id_str = id_str.encode('utf-8')
        if re.match('^\d+(_\d+)*$', id_str) is None:
            raise web.BadRequest("Invalid input config export id_str")
        else:
            config_list = ConfigAdmin.get_all_configs()
            id_str_list = id_str.split('_')
            result_list = []
            for id_str in id_str_list:
                for config in config_list:
                    if config['id'] == int(id_str):
                        result_list.append(config)
                        break

            export_path = "/tmp/export_config"
            clear_path(export_path)
            try:
                config_name = "config"
                file_content = json.dumps(result_list, indent=1, ensure_ascii=False)
                file_path = "/tmp/export_config/%s.json" % config_name
                write_file(file_path, file_content)

                # 压缩日志
                dt = datetime.now()
                export_config_file_path = "/tmp/" + config_name + "-" + dt.strftime(
                    '%Y-%m-%d-%H-%M-%S') + ".config_export"

                record_dir = os.getcwd()
                cmd_str = 'tar cvzf %s *' % export_config_file_path
                os.chdir(export_path)
                print(cmd_str)
                os.system(cmd_str)
                os.chdir(record_dir)
                os.system('sync')
                return send_file(export_config_file_path)
            except IndexError:
                raise web.internalerror("Index error")
            except Exception:
                raise web.internalerror()
            # finally: # 删除会导致文件还没发送就删除了
            #     os.remove(file_path)

    def OPTIONS(self):
        pass


class API_ConfigInfoRollback:
    def GET(self):
        print("Rollback configs")
        file_path = "/sros/backup/configs/config_backup.config_export"
        if os.path.isfile(file_path):
            is_exist_new_config_info = False
            try:
                is_exist_new_config_info = get_files_update_configs(file_path, "/tmp/rollback_config", 1)
            except Exception as e:
                print(e)

            return json.dumps({"state": True, "code": 9999, "message": "",
                               "data": {"is_exist_new_config_info": is_exist_new_config_info}})
        return web.BadRequest("No configs backup file")


class API_ConfigInfoImport:
    def POST(self):
        print("Import configs")
        try:
            x = web.input(file={})
            client_md5 = str(web.input(md5='').md5)
            is_override = int(web.input(override='0').override)
        except ValueError:
            raise web.NotAcceptable("Import config file error")

        file_path = "/tmp/import_config.tar.gz"
        if 'file' in x:
            print("Import config file:", x['file'].filename)

            with open(file_path, 'wb') as tmp_file:
                tmp_file.write(x['file'].file.read())
                tmp_file.close()
            os.system('sync')
            if not check_file_md5(file_path, client_md5):
                raise web.BadRequest('file md5 check error')

            backup_configs()

            is_exist_new_config_info = get_files_update_configs(file_path, "/tmp/import_config", is_override)

            return json.dumps({"state": True, "code": 9999, "message": "",
                               "data": {"is_exist_new_config_info": is_exist_new_config_info}})
        else:
            return web.BadRequest("No file content")

    def OPTIONS(self):
        pass


class API_SystemUpgradeRollback:
    def GET(self):
        try:
            print("sros rollback")
            path_target = "/sros/bin"
            d = os.path.expanduser(path_target)
            files = [os.path.join(d, f) for f in os.listdir(d)]
            sros_files = []
            for file_name in files:
                if file_name.find("/sros_", 9) > 0:
                    sros_files.append(file_name)
            sros_files.sort(key=lambda f: os.stat(f).st_ctime)
            if len(sros_files) < 2:
                return json.dumps({"state": False, "code": 9999, "message": "", "data": {}})
            last_modified = sros_files[-2]
            cmd = "ln -fs " + last_modified + " " + path_target + "/sros"
            print cmd
            os.system(cmd)
            os.system("systemctl stop sros")
            os.system("systemctl start sros")
            return json.dumps({"state": True, "code": 9999, "message": "", "data": {}})
        except Exception as e:
            _logger.error("API_SystemUpgradeRollback", e)
            raise web.internalerror(e.message)


def customhook():
    web.header('Access-Control-Allow-Origin', '*')
    web.header('Access-Control-Allow-Headers',
               'Content-Type, Access-Control-Allow-Origin, Access-Control-Allow-Headers, X-Requested-By, Access-Control-Allow-Methods')
    web.header('Access-Control-Allow-Methods', 'POST, GET, PUT, DELETE')


class API_Error_Log:
    def __init__(self):
        self.connect = sqlite3.connect('/sros/db/main.db3')  # fix
        self.cursor = self.connect.cursor()

    def GET(self):
        curPage = web.input(curPage=0).curPage  # 从多少行开始获取
        pageSize = web.input(pageSize=10).pageSize  # 获取多少行，取值范围[1, 1000]
        timeBegin = web.input(timeBegin=0).timeBegin
        timeEnd = web.input(timeEnd=2000000000).timeEnd
        type = web.input(type="0123").type
        level = web.input(level="1234").level
        curPage = int(curPage)
        pageSize = int(pageSize)
        timeBegin = str(timeBegin)
        timeEnd = str(timeEnd)
        pageSize = max(1, min(pageSize, 1000))
        curPage = max(0, curPage)
        query = "select * from error_log where"
        timecond = " time_stamp_int>=" + timeBegin + " and " + "time_stamp_int<=" + timeEnd
        query += timecond
        Typecond = self.getQuerycond(type, "type")
        if (len(Typecond) <= 1):
            self.connect.close()
            raise Exception("type is empty")
        Levelcond = self.getQuerycond(level, "level")
        if (len(Levelcond) <= 1):
            self.connect.close()
            raise Exception("type is empty")
        query = query + " and " + Typecond + " and " + Levelcond + " order by id ASC"
        # query = query + Typecond + " and " + Levelcond
        try:
            self.cursor.execute(query)
            values = self.cursor.fetchall()
            self.connect.close()
            return self.getDataFromQueryResult(values, curPage, pageSize)
            # self.connect.commit() #what is it
        except Exception as e:
            self.connect.close()
            raise Exception("query:%s error", query)

    def getDataFromQueryResult(self, values, curPage, pageSize):
        ret = {}
        content = []
        indexBegin = curPage * pageSize
        indexEnd = indexBegin + pageSize
        while indexBegin < indexEnd and indexBegin < len(values):
            tmp = {}
            tmp["id"] = values[indexBegin][0]
            tmp["error_id"] = values[indexBegin][3]
            tmp["level"] = values[indexBegin][1]
            tmp["type"] = values[indexBegin][2]
            tmp["taskId"] = values[indexBegin][4]
            tmp["detail"] = values[indexBegin][5]
            tmp["description"] = values[indexBegin][6]
            tmp["solve"] = values[indexBegin][7]
            tmp["occurTimerstamp"] = values[indexBegin][9]
            content.append(tmp)
            indexBegin += 1
        ret["state"] = True
        ret["code"] = 9999
        ret["msg"] = 'success'
        ret["total"] = len(values)
        ret["curPage"] = curPage
        ret["pageSize"] = pageSize
        ret["result"] = content
        return json.dumps(ret)

    def getQuerycond(self, inputStr, key):
        ret = "("
        for i in range(0, 5):
            if inputStr.find(str(i)) != -1:
                if len(ret) > 1:
                    ret += " or "
                ret += key
                ret += "="
                ret += str(i)
        if len(ret) > 1:
            ret += ")"
        return ret


if __name__ == "__main__":
    app = web.application(urls, globals())
    app.add_processor(web.loadhook(customhook))
    app.run()
