# -*- coding:utf-8 -*-
# file utils.py
# author YangHuaiDong
# date 2021/10/27 上午9:27
# copyright Copyright (c) 2021 Standard Robots Co., Ltd. All rights reserved.
# describe
import json
import os
import web
import time
import hashlib
import shutil
from datetime import datetime
from sros_log import _logger

from database import (
    db_main as db,
    AccountAdmin,
    ConfigAdmin,
    TaskTemplateAdmin,
    UserMissionAdmin,
    MapsAdmin,
)

SROS_MAP_DIR = "/sros/map/"
SROS_TOOL_DIR = "/sros/tool/"
SROS_PIPE_FILE = "/tmp/sros_pipe_read_fifo"

db = web.database(dbn='sqlite', db="/sros/db/main.db3")
BUF_SIZE = 1024

# 如果管道文件不存在，则需等待管道建立
def create_pipe():
    while True:
        if (not os.path.exists(SROS_PIPE_FILE)):
            time.sleep(1)
            continue
        else:
            try:
                read_fifo = os.open(SROS_PIPE_FILE, os.O_WRONLY)
                _logger.info("open %s success"% SROS_PIPE_FILE);
                break
            except IOError as e:
                _logger.error("open %s failed"% SROS_PIPE_FILE);
                break;
    return read_fifo

# 初始化并创建与SROS读写管道
sros_read_fifo = create_pipe()

def read_file(file_path):
    if file_path == "" or not os.path.exists(file_path):
        raise web.internalerror("file not exist: " + file_path)
    with open(file_path, 'r') as file_handle:
        file_content = file_handle.read()
    return file_content


def write_file(file_path, file_content):
    if file_path == "":
        raise web.internalerror("file not exist: " + file_path)
    with open(file_path, 'w') as file_handle:
        file_handle.write(file_content)
        file_handle.close()
    return True


def timestamp_to_timestr(timestamp):
    if timestamp == '' or timestamp == 0:
        return 'NA'
    else:
        t = time.localtime(float(timestamp))
        return time.strftime('%Y-%m-%d %H:%M:%S', t)


def md5sum(filename):
    with open(filename, 'rb') as f:
        d = hashlib.md5()
        d.update(f.read())
        md5 = d.hexdigest()
        f.close()
        return md5


def check_file_md5(file_path, rcv_md5):
    if rcv_md5 == '':
        return True
    if not os.path.exists(file_path):
        return False
    server_md5 = md5sum(file_path)
    if server_md5 != rcv_md5:
        print('file md5 mismatch: client_md5=' + rcv_md5 + ' server_md5=' + server_md5)
        return False
    return True


def get_settings_list():
    rs = db.select('config')

    ds = dict()

    for r in rs:
        key = r['key']
        ks = key.split('.')

        sec = ks[0].encode('ascii', 'ignore')
        if sec not in ds:
            ds[sec] = dict()
        else:
            ds[sec][r['id']] = r
            ds[sec][r['id']]['changed_time'] = timestamp_to_timestr(r['changed_time'])

    return ds


def get_map_name_list(map_dir):
    map_name_list = []
    for dirpath, dirnames, filenames in os.walk(map_dir):
        for file_name in filenames:
            if file_name[-4:] == ".map":
                map_name = file_name[:-4]
                json_path = os.path.join(map_dir, map_name + ".json")
                if not os.path.exists(json_path):
                    create_map_json(map_name)
                map_name_list.append(map_name)
    return map_name_list


def get_map_item_info(map_name, map_dir):
    full_path = os.path.join(map_dir, map_name + ".json")
    full_path_map = os.path.join(map_dir, map_name + ".map")
    if os.path.exists(full_path_map) == True and os.path.exists(full_path) == False:
        create_map_json(map_name) #新建地图时，sros不会创建json文件，需要python

    # t = os.path.getmtime(unicode(full_path, 'utf8'))
    t = os.path.getmtime(full_path)
    modify_time = 0
    if t != '':
        modify_time = t * 1000
    map_item = dict(name=map_name,
                    md5=md5sum(full_path),
                    modify_time=modify_time
                    )
    return map_item


def get_map_item_meta(map_name):
    result = {}
    json_path = os.path.join(SROS_MAP_DIR, map_name + ".json")
    if not os.path.exists(json_path):
        raise web.internalerror("map json file not exist")
    json_file_md5 = md5sum(json_path)
    result["name"] = map_name
    result["md5"] = json_file_md5
    with open(json_path, 'r') as file_handle:
        file_content = file_handle.read()
        map_json = json.loads(file_content)
        for key, value in map_json["meta"].iteritems():
            if key == "length_unit" or key == "angle_unit":
                continue
            result[key] = value
        # for key, value in map_json["data"].iteritems():
        #     if type(value) == list:
        #         result[key] = len(value)
    result["modified_timestamp"] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(os.path.getmtime(json_path)))
    return result


def get_map_item_json(map_name, map_dir):
    json_path = os.path.join(map_dir, map_name + ".json")

    if os.path.exists(json_path):
        return json.load(open(json_path, 'r'))
    else:
        return {"meta": {}, "data": {}}


def get_map_file_path(map_name, file_type, map_dir=SROS_MAP_DIR):
    if file_type == 'map':
        map_file_path = os.path.join(map_dir, map_name + ".map")
    else:
        map_file_path = os.path.join(map_dir, map_name + ".json")
    if not os.path.exists(map_file_path):
        return ""
    return map_file_path


def create_map_image(map_name):
    if map_name == "":
        raise web.BadRequest()

    if map_name not in get_map_name_list(map_dir=SROS_MAP_DIR):
        raise web.NotFound()

    export_png_tool = SROS_TOOL_DIR + "sros_export_png_map"
    src_map_file = os.path.join(SROS_MAP_DIR, map_name + ".map")
    # dst_png_file = os.path.join(SROS_MAP_DIR, map_name + ".png")
    if os.path.exists(export_png_tool):
        cmd_str = "%s %s %s" % (export_png_tool, src_map_file, map_name)
        # print cmd_str
        os.system(cmd_str)


def create_map_json(map_name):
    if map_name == "":
        raise web.BadRequest()
    json_path = os.path.join(SROS_MAP_DIR, map_name + ".json")
    map_path = os.path.join(SROS_MAP_DIR, map_name + ".map")
    cmd = SROS_TOOL_DIR + "sros_create_map_json " + map_path + " " + json_path + " "
    os.system(cmd)


def create_mission(mission):
    account_id = mission["owner_id"]
    rs = UserMissionAdmin.create_mission(mission)
    # print mission

    # 管理员和开发者可以访问所有mission,不需要保存到数据库
    # 普通用户无法新建任务，这部分代码应该没意义了
    account = AccountAdmin.get(account_id)
    if int(account['permission']) < 20:
        UserMissionAdmin.add_mission(account_id, int(rs))
    # print rs
    return rs


def update_map(map_name, json_dat):
    if map_name == "" or not json_dat:
        return False
    json_str = json.dumps(json_dat, indent=1, ensure_ascii=False)
    map_json_file_path = "/sros/map/%s_tmp.json" % map_name
    map_file_path = "/sros/map/%s.map" % map_name
    f = open(map_json_file_path, 'w')
    f.write(json_str)
    f.close()
    cmd = SROS_TOOL_DIR + "sros_map_json_to_map " + map_json_file_path + " " + map_file_path + " "
    ret = os.system(cmd)
    if ret == 0:
        send_pipe_dat('update_map ' + map_name + '\n')
        print("%s write to pipe for update map" % map_name)
        return True
    return False


def send_pipe_dat(dat):
    try:
        os.write(sros_read_fifo, dat)
    except IOError as e:
        print(e.message)


def send_file(file_path):
    file_path = file_path.encode('utf-8')
    print('file_path is ' + file_path)
    if not os.path.exists(file_path):
        raise web.NotFound()
    try:
        f = open(file_path, "rb")
        web.header('Content-Type', 'application/octet-stream')
        web.header('Content-Length', os.path.getsize(file_path))
        web.header('Content-MD5', md5sum(file_path))
        web.header('Content-disposition', 'attachment; filename=%s' % os.path.basename(file_path))
        while True:
            c = f.read(BUF_SIZE)
            if c:
                yield c
                # time.sleep(0.001)  # 限速10M
            else:
                break
    except Exception as e:
        print(e)
        yield 'Error'
    finally:
        if f:
            f.close()


def clear_path(file_path):
    if os.path.exists(file_path):
        shutil.rmtree(file_path)
    os.mkdir(file_path)


def get_export_task_file_path():
    return "/tmp/export_task"


def get_import_task_file_path():
    return "/tmp/import_task"


def get_files_from_path(file_path, filter_ext=""):
    files = []
    for file_item in os.listdir(file_path):
        path = os.path.join(file_path, file_item)
        file_name, ext = os.path.splitext(path)
        if os.path.isfile(path):
            if filter_ext == "" or (filter_ext != "" and ext == filter_ext):
                files.append(path)
        elif os.path.isdir(file_item):
            files.extend(get_files_from_path(path))
    return files


# 导出任务时，将任务及所有子任务写入json文件，并返回所有文件的字符串，用于压缩
def create_task_json_file(task, is_template=False):
    task_name = task["name"].encode('utf-8')
    task["body"] = json.loads(task["body"])
    if "item_valid" in task.keys():
        del task["item_valid"]

    # 处理任务嵌套
    for k, v in task["body"].items():
        step_info = v["stepInfo"]
        if step_info["stepType"] == "mission":
            sub_mission_id = step_info["missionId"]
            if is_template:
                sub_task = TaskTemplateAdmin.get_template(sub_mission_id)
                sub_task["body"] = json.dumps(sub_task["body"])
            else:
                sub_task = UserMissionAdmin.get_detailed_mission(sub_mission_id)
            if not sub_task:
                print("Failed to find task " + str(sub_mission_id))
                continue
            create_task_json_file(sub_task, is_template)

    file_content = json.dumps(task, indent=1, ensure_ascii=False)
    file_path = "/tmp/export_task/%s.json" % task_name
    write_file(file_path, file_content)


# 更新任务与子任务的绑定关系
def update_subtask_id(map_task_ids, is_template=False):
    for old_id, new_id in map_task_ids.items():
        if is_template:
            task = TaskTemplateAdmin.get_template(new_id)
        else:
            task = UserMissionAdmin.get_detailed_mission(new_id)
            task["body"] = json.loads(task["body"])
            task["mapName"] = task["map_name"]

        for k, v in task["body"].items():
            step_info = v["stepInfo"]
            if step_info["stepType"] == "mission" and step_info["missionId"] in map_task_ids.keys():
                old_sub_task_id = step_info["missionId"]
                step_info["missionId"] = map_task_ids[old_sub_task_id]
                if is_template:
                    # print("update template id " + str(old_sub_task_id) + " to " + str(map_task_ids[old_sub_task_id]))
                    task["body"] = json.dumps(task["body"])
                    TaskTemplateAdmin.update_template(task)
                else:
                    # print("update task id " + str(old_sub_task_id) + " to " + str(map_task_ids[old_sub_task_id]))
                    UserMissionAdmin.update_detailed_mission(task)


def authenticated(method):
    def wrapper(*args, **kwargs):
        # TODO 权限验证
        # session_id = web.input().get("session")
        # if session_id is None:
        #     raise web.unauthorized()
        return method(*args, **kwargs)

    return wrapper


def generate_map_export(map_dir, map_name, type='FMS'):
    """
    type类型为FMS/OM, FMS -- 导航地图，OM　--　修图
    """
    assert (type == 'FMS' or type == 'OM'), '未知type: {0}'.format(type)
    suffixes = ['.map', '0.pgm', '1.pgm', '2.pgm', '_update0.pgm', '_update1.pgm', '_update2.pgm', '.json', '.png',
                '_2.png', '_4.png', '_8.png', '.lom', '.region', '.info']
    if type == 'OM':
        suffixes += ['.om', '.g2o']

    # 新增双图层文件后缀
    suffixes += ['_10.pgm', '_11.pgm', '_12.pgm'];
    suffixes += ['_1.lom', '_2.lom'];
    suffixes += ['_1.region', '_2.region'];

    file_name_str = ""
    for suffix in suffixes:
        file_name = os.path.join(map_dir, map_name + suffix)
        if os.path.exists(file_name):  # 可能存在部分文件不存在的情况，比如缺少'.json'文件
            file_name_str += " " + file_name

    return file_name_str


# 备份configs
def backup_configs():
    print('backup configs')
    config_list = ConfigAdmin.get_all_configs()
    export_path = "/sros/backup/configs"

    if os.path.exists(export_path):
        shutil.rmtree(export_path)
    os.makedirs(export_path)

    try:
        file_content = json.dumps(config_list, indent=1, ensure_ascii=False)
        file_path = "/sros/backup/configs/config.json"
        write_file(file_path, file_content)

        # 压缩日志
        dt = datetime.now()
        export_config_file_path = "/sros/backup/configs/config_backup.config_export"

        record_dir = os.getcwd()
        cmd_str = 'tar cvzf %s config.json' % export_config_file_path
        os.chdir(export_path)
        print(cmd_str)
        os.system(cmd_str)
        os.chdir(record_dir)
        cmd_rm_str = 'rm %s' % file_path
        print(cmd_rm_str)
        os.system(cmd_rm_str)
        os.system('sync')
        return True
    except IndexError:
        raise web.internalerror("Index error")
    except Exception:
        raise web.internalerror()
    # finally: # 删除会导致文件还没发送就删除了
    #     os.remove(file_path)


# 更新config
def get_files_update_configs(file_path, extract_path, is_override):
    print("get_files_update_configs", file_path, extract_path, is_override)
    try:
        clear_path(extract_path)

        cmd_str = 'tar zxvf %s -C %s' % (file_path, extract_path)
        print(cmd_str)
        os.system(cmd_str)
        os.system('sync')

        is_exist_new_config_info = False
        config_files = get_files_from_path(extract_path, ".json")
        # print(config_files)
        for config_file in config_files:
            file_content = read_file(config_file)
            config_json = json.loads(file_content)

            is_exist_new_config_info = update_configs(is_override, config_json)

    except Exception as e:
        print(e)
        raise web.badrequest("configs file format error")
    send_pipe_dat('update_db_config ' + "/nothing" + '\n')
    return is_exist_new_config_info


def update_configs(is_override, config_new_list=[]):
    is_exist_new_config_info = False
    for config_new in config_new_list:
        config_old = ConfigAdmin.get_config_by_id(config_new['id'])
        if config_old is None:
            is_exist_new_config_info = True
        else:
            if is_override:
                ConfigAdmin.update_config(True, config_old, config_new)
    return is_exist_new_config_info
