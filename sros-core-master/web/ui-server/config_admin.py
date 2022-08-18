# -*- coding: utf-8 -*-

import json
import web
import csv
import os

from database import ConfigAdmin
from database import TaskTemplateAdmin

# 数据库连接配置，host=数据库服务器地址，user=用户名，pw=密码，db=数据库名称
db_main = web.database(dbn='sqlite', db="/sros/db/main.db3")

LANG_CN = 'zh-cn'
LANG_EN = 'en-us'
LANG_JA = 'ja'
LANG_HAN = 'zh-hant'


def read_csv_file(file_path, skip_header=False):
    # print file_path
    if not os.path.exists(file_path):
        return None
    with open(file_path) as f:
        csv_reader = csv.reader(f)
        if skip_header:
            next(csv_reader, None)
        rows = []
        for row in csv_reader:
            rows.append(row)
    f.close()
    return rows


def read_file(file_path):
    if file_path == "" or not os.path.exists(file_path):
        raise web.internalerror("file not exist: " + file_path)
    with open(file_path, 'r') as file_handle:
        file_content = file_handle.read()
    return file_content

def read_json_file(file_path):
    with open(file_path,'r') as json_f:
            json_dict = json.load(json_f)
    return json_dict

def get_config_file_path(file_name):
    return '/sros/web/ui-server/configs/' + file_name
    # return './configs/' + file_name


def get_config_error_file_path(file_name):
    return '/sros/cfg/resources/' + file_name


def get_config_sensor_color_display_path(file_name):
    return '/sros/cfg/resources/device/' + file_name

#将cn_item中的中文描述替换为i18n_dict中的对应的lang类型的语言描述
def replace_i18n_language_desc(cn_item,i18n_dict,keyname,lang):
    for item in i18n_dict:
        if item[keyname] == cn_item['id']:
            for obj in item['i18n']:
                if obj['language'] == lang:
                    cn_item['desc'] = obj['describe']
                    cn_item['how'] = obj['how_to_fix']

class API_ConfigAdmin:
    def __init__(self):
        self.init()
        return

    def init(self):
        # 初始化数据库，写入默认充电任务
        if TaskTemplateAdmin.get_template_by_name('charge') is None:
            charge_config_file = get_config_file_path('default_charge_task.json')
            json_charge = json.loads(read_file(charge_config_file))
            tpl_charge = {"name": "charge", "desc": "charge", "owner_id": 3, "body": json.dumps(json_charge["body"])}
            TaskTemplateAdmin.create_template(tpl_charge)

    def GET(self):
        file_type = web.input(type='').type
        lang = web.input(lang=LANG_CN).lang
        if file_type == 'action_table':
            file_path = get_config_file_path('action_table.csv')
            if lang == LANG_EN:
                file_path = get_config_file_path('action_table_en.csv')
            elif lang == LANG_HAN:
                file_path = get_config_file_path('action_table_zh_hant.csv')
            elif lang == LANG_JA:
                file_path = get_config_file_path('action_table_ja.csv')
            ret = self.get_action_list(file_path)
            if not ret:
                web.internalerror("failed to read config file")
            return ret
        elif file_type == 'preference_config':
            ret = self.get_dynamic_preference(get_config_file_path("dynamic_preference.csv"))
            if not ret:
                web.internalerror("failed to read perference config file")
            return ret
        elif file_type == 'modbus_register':
            ret = self.get_modbus_register()
            if not ret:
                web.internalerror("failed to read modbus register config files")
            return ret
        elif file_type == 'charge_task':
            return json.dumps(self.get_default_charge_task())
        elif file_type == 'error_code':
            return json.dumps(self.get_error_codes(lang), encoding="UTF-8", ensure_ascii=False)
        elif file_type == 'sensor_color_display':
            ret = self.get_sensor_color_display()
            if not ret:
                web.internalerror("failed to read sensor color display config files")
            return ret
        elif file_type == 'user_defined_properties':
            ret = self.get_user_defined_properties()
            if not ret:
                web.internalerror("failed to read config_web user_defined_properties files")
            return ret
        return web.NotFound('Unsupport input parameter')

        # 如果不重写，post会报错

    def OPTIONS(self, user_id):
        pass

    def get_action_list(self, file_path):
        file_rows = read_csv_file(file_path)
        if not file_rows:
            return None
        action_list = dict()
        map_action_id = self.get_map_action_id()
        for row in file_rows:
            if len(row) < 6:
                continue
            action = dict(zip(['id', 'param0', 'param1', 'type', 'name', 'desc'], row))
            id = action['id']
            if int(id) != map_action_id and int(id) < 65:
                continue
            if action['desc'] == '':
                if id not in action_list:
                    action_list[id] = []
                continue
            if id not in action_list:
                print 'action table format error: ' + id
                continue
            action_list[id].append(action)
        # 编码转换，否则中文乱码
        return json.dumps(action_list, encoding="UTF-8", ensure_ascii=False)

    def get_dynamic_preference(self, file_path):
        file_rows = read_csv_file(file_path)
        prefer_groups = []
        if not file_rows:
            return None
        for row in file_rows:
            if len(row) != 2:
                continue
            key = row[1].strip()
            if key == '':
                continue
            dot_index = key.find(".")
            if dot_index < 0:
                continue
            group_name = key[0:dot_index]
            group = self.find_group(group_name, prefer_groups)
            if not group:
                group = {"title": group_name, "data": []}
                prefer_groups.append(group)
            config_list = list(db_main.select('config', where='key = "%s"' % key))
            if len(config_list) <= 0:
                continue
            # 命名方式转为驼峰
            self.normalize_keys(config_list[0])
            group["data"].append(config_list[0])
        # 编码转换，否则中文乱码
        return json.dumps(prefer_groups, encoding="UTF-8", ensure_ascii=False)

    def get_modbus_register(self):
        file_rows = read_csv_file(get_config_file_path('modbus_register.csv'), True)
        registers = []
        if not file_rows:
            return None
        for row in file_rows:
            if len(row) != 5:
                continue
            reg_key = row[0]  # 前端根据key从多语言文件获取名称
            reg_type = int(row[1])
            reg_addr = int(row[2], 16)
            reg_property = int(row[3])
            reg_num = int(row[4])
            registers.append({"addr": reg_addr, "key": reg_key, "type": reg_type, "quantity": reg_num, "property": reg_property})
        return json.dumps(registers, encoding="UTF-8", ensure_ascii=False)

    def get_sensor_color_display(self):
        file_path = get_config_sensor_color_display_path('sensor_color_display.csv')

        file_rows = read_csv_file(file_path, True)
        if not file_rows:
            return {}

        sensor_color_list = list()
        for row in file_rows:
            if len(row) < 5:
                continue
            sensor_color_item = dict(zip(['id', 'sensor_name', 'sensor_desc', 'color', 'oba_type'], row))
            sensor_color_item["id"] = int(sensor_color_item["id"])
            sensor_color_list.append(sensor_color_item)

        return json.dumps(sensor_color_list, encoding="UTF-8", ensure_ascii=False)

    def get_user_defined_properties(self):
        file_path = get_config_file_path('user_defined_properties.csv')

        file_rows = read_csv_file(file_path, True)
        if not file_rows:
            return []

        properties_list = list()
        for row in file_rows:
            if len(row) < 5:
                continue
            item = dict(zip(['id', 'type', 'title', 'key', 'key_val'], row))
            item["id"] = int(item["id"])
            item["type"] = int(item["type"])
            properties_list.append(item)

        return json.dumps(properties_list, encoding="UTF-8", ensure_ascii=False)

    def get_error_codes(self, lang=LANG_CN):
        file_path_error = get_config_error_file_path('error_code.csv')
        file_path_fault = get_config_error_file_path('fault_code.csv')
        
        #国际化json读取
        i18n_error_file_path = get_config_error_file_path('error_code_i18n.json')
        i18n_fault_file_path = get_config_error_file_path('fault_code_i18n.json')
        i18n_error_dict=read_json_file(i18n_error_file_path)
        i18n_fault_dict=read_json_file(i18n_fault_file_path)

        file_rows = read_csv_file(file_path_error, True)
        if not file_rows:
            return {}
        error_list = list()
        for row in file_rows:
            if len(row) < 4:
                continue
            error_item = dict(zip(['id', 'name', 'desc', 'how'], row))
            error_item["id"] = int(error_item["id"])
            if lang != LANG_CN:
                replace_i18n_language_desc(error_item,i18n_error_dict,'error_code',lang)
            error_list.append(error_item)

        file_rows = read_csv_file(file_path_fault, True)
        if not file_rows:
            return {}
        fault_list = list()
        for row in file_rows:
            if len(row) < 6:
                continue
            fault_item = dict(zip(['id', 'device', 'level', 'name', 'desc', 'how', 'music'], row))
            fault_item["id"] = int(fault_item["id"])
            if lang != LANG_CN:
                replace_i18n_language_desc(fault_item,i18n_fault_dict,'fault_code',lang)    
            fault_list.append(fault_item)
        return {'error': error_list, 'fault': fault_list}

    def get_default_charge_task(self):
        file_content = read_file(get_config_file_path("default_charge_task.json"))
        return json.loads(file_content, encoding="UTF-8")

    def find_group(self, group_name, group_list):
        for group in group_list:
            if group.get("title", "") == group_name:
                return group
        return None

    def normalize_keys(self, obj):
        for (k, v) in obj.items():
            if k == "default_value":
                del obj[k]
                obj["defaultValue"] = v
            elif k == "value_units":
                del obj[k]
                obj["valueUnit"] = v
            elif k == "value_type":
                del obj[k]
                obj["valueType"] = v
            elif k == "value_range":
                del obj[k]
                obj["valueRange"] = v
            elif k == "is_valid":
                del obj[k]
                obj["valid"] = v

    def set_keyname_camels(self, obj):
        for (k, v) in obj.items():
            new_k = self.to_camels(k)
            del obj[k]
            obj[new_k] = v
        return obj

    def to_camels(self, str):
        if "_" in str:
            str_list = str.split("_")
            str0 = str_list[0]
            str_list.remove(str0)
            for s in str_list:
                s = s.capitalize()
                str0 += s
            return str0
        return str

    # 根据配置重读取到的车型映射到动作
    def get_map_action_id(self):
        vehicle_type = ConfigAdmin.get_config('main.vehicle_type')
        action_id = 1
        if not vehicle_type:
            return action_id
        action_unit = ConfigAdmin.get_config('main.action_unit')
        if not action_unit:
            return action_id

        vehicle_str = vehicle_type.value
        action_str = action_unit.value
        if vehicle_str == 'oasis':
            if action_str == 'riser':
                action_id = 1
            elif action_str == 'droller':
                action_id = 2
            elif action_str == 'roller':
                action_id = 3
            elif action_str == 'rise-rotater':
                action_id = 4
        return action_id
