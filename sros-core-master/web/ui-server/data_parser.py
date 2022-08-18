# -*- coding: utf-8 -*-

import web
import json
import time
import os
import struct

import sys

reload(sys)
sys.setdefaultencoding('utf-8')

BUFF_SIZE = 2048

# 基本时间步长单位, 3s
TIME_STEP_BASE_SECONDS = 3

# 每个字段最多返回3000个点数据
MAX_RESULT_DATA_LEN = 3000

SROS_MONITOR_DIR = "/sros/monitor"
MAP_DIR = "/sros/map"
MONITOR_FILE_TO_JSON_COMMAND = '/sros/tool/sros_monitor_file_to_json'
KEYWORD_FEATURE_NODE = "FEATURE_NODE"
KEYWORD_SPARSE_LINK_MSG = "SPARSE_LMK_MSG"


def read_file(file_path):
    if file_path == "" or not os.path.exists(file_path):
        raise IOError("file not exist: " + file_path)
        return ""
    with open(file_path, 'r') as file_handle:
        file_content = file_handle.read()
    return file_content


def write_file(file_path, file_content):
    if file_path == "":
        raise web.internalerror("file not exist: " + file_path)
        return False
    with open(file_path, 'w') as file_handle:
        file_handle.write(file_content)
        file_handle.close()
    return True


class LMKParser:
    def __init__(self):
        pass

    @staticmethod
    def get_lmk_list(map_name):
        lmk_list = []
        if map_name == "":
            return lmk_list
        lom_file = MAP_DIR + "/" + map_name + ".lom"
        # lom_file = './temp/20190412.lom'
        if not os.path.exists(lom_file):
            print "file path not exist: ", lom_file
            return lmk_list
        with open(lom_file, 'rt') as tmp_file:
            for line in tmp_file:
                if KEYWORD_FEATURE_NODE in line:
                    items = line.split()
                    if len(items) < 8 or items[0] != KEYWORD_FEATURE_NODE:
                        continue
                    if int(items[1]) != 3:
                        continue
                    lmk_list.append({"x": float(items[3]) * 100, "y": float(items[4]) * 100, "type": int(items[7])})
                elif KEYWORD_SPARSE_LINK_MSG in line:
                    items = line.split()
                    if len(items) < 4 or items[0] != KEYWORD_SPARSE_LINK_MSG:
                        continue
                    lmk_list.append({"x": float(items[1]) * 100, "y": float(items[2]) * 100, "type": int(items[3])})
        return lmk_list


class QRCodeParser:
    def __init__(self):
        pass

    @staticmethod
    def get_qrcode_list(map_name):
        qrcode_list = []
        if map_name is None:
            return qrcode_list
        try:
            file_path = MAP_DIR + "/" + map_name + ".info"
            file_content = read_file(file_path)
            dat = json.loads(file_content, encoding="utf-8")
            dm_code = dat["dm_code"]
            location = dm_code["location"]
            for key in location:
                info = location[key]
                qrcode_list.append({
                    "dmcode_id": info["code_id"],
                    "x": float(info["world_pose"]["x"]),
                    "y": float(info["world_pose"]["y"]),
                    "yaw": float(info["world_pose"]["yaw"]),
                    "weight": int(int(info["weight"])),
                    "recorded": info["recorded"] == "True",
                    "damaged": info["damaged"] == "True"
                })
        except BaseException as e:
            print("parse qrcode file error")
            print(e)
        return qrcode_list

    @staticmethod
    def set_qrcode_list(qrcode_front):
        file_path = MAP_DIR + "/" + qrcode_front["mapName"].encode("utf-8") + ".info"
        if not os.path.exists(file_path):
            return True
        try:
            file_content = read_file(file_path)
            dat = json.loads(file_content, encoding="utf-8")
            location_temp = {}
            location_dat = dat["dm_code"]["location"] or {}
            for info in qrcode_front["fuseLocateQRCode"]:
                qrcode_id = info["dmcode_id"]
                location_temp[qrcode_id] = location_dat[qrcode_id]
            dat["dm_code"]["location"] = location_temp
            file_content_2 = json.dumps(dat, indent=1, ensure_ascii=False)
            return write_file(file_path, file_content_2)
        except BaseException as e:
            print("parse qrcode file error when set_qrcode_list")
            print(e)
            return False


class MonitorParser:
    def __init__(self):
        pass

    def parse_from_to(self, start_timestamp=0, end_timestamp=int(round(time.time() * 1000)), dat_items=[]):
        print "parse monitor from ", start_timestamp, " to ", end_timestamp
        monitor_files = self.get_monitor_files(start_timestamp, end_timestamp)
        # print monitor_files
        file_list = []
        block_dat = []
        json_files = []
        for timestamp, file_path in monitor_files:
            file_base, ext = os.path.splitext(file_path)
            json_file = file_base + ".json"
            json_files.append(json_file)
            # 如果json文件不存在，或者即使.mf文件的时间戳大于json文件的时间戳（.mf文件有更新）
            # 则需要生成json文件
            if not os.path.exists(json_file) or (os.path.exists(json_file) and (os.path.getmtime(file_path) > os.path.getmtime(json_file))):
                file_list.append(file_path)

        if len(file_list) > 0:
            self.create_json_files(file_list)

        for json_file in json_files:
            dat = self.parse_json_file(json_file)
            block_dat.extend(dat)
        return self.get_only_specified_data(dat_items, block_dat)

    def create_json_files(self, monitor_files):
        # 转为json文件
        spit_char = ','
        cmd_arg = spit_char.join(monitor_files)
        ret = os.system(MONITOR_FILE_TO_JSON_COMMAND + ' ' + cmd_arg)
        if ret != 0:
            print "run ", MONITOR_FILE_TO_JSON_COMMAND, " error"
            return False
        return True

    def parse_json_file(self, file_path):
        print 'Parse json file', file_path
        if file_path == '':
            return
        if not os.path.exists(file_path):
            print 'File path not exist:', file_path
            return []
        with open(file_path, 'rt') as tmp_file:
            json_dat = json.loads(tmp_file.read())
        all_records = json_dat['records']

        # 由于采样时每秒一次,如果全部传到前端,数据量过大,先做一级过滤
        # 每3s取一个数据
        sample_record = []
        for index in range(0, len(all_records), TIME_STEP_BASE_SECONDS):
            # 取中间数据，之所以没取平均值，是因为数据对象是dict，需要对所有字段分别取平均值
            sample_record.append(all_records[index])
        return sample_record

    def get_monitor_files(self, from_time, to_time):
        # 已经按时间戳排序
        monitor_files = self.get_all_monitor_files()
        # 由于一个小时内创建一个文件，请求的时间在最近创建文件之后
        # 此时应该返回最后一个文件
        start_index = len(monitor_files) - 1
        end_index = len(monitor_files)
        find_end = False
        for index in range(len(monitor_files)):
            timestamp, file_path = monitor_files[index]
            if from_time >= timestamp:
                # 往前读取一个文件，因为部分数据可能在上一个文件中
                start_index = index
            if to_time <= timestamp and not find_end:
                find_end = True
                end_index = index
        return monitor_files[start_index:end_index]

    # 已经按时间戳排序
    def get_all_monitor_files(self):
        file_lst = {}
        for root, dirs, files in os.walk(SROS_MONITOR_DIR):
            for file_name in files:
                file_path = root + '/' + file_name
                if os.path.splitext(file_path)[1] == '.mf':
                    timestamp = file_name.replace('m_', '').replace('.mf', '')
                    file_lst[int(timestamp)] = file_path
        return [(k, file_lst[k]) for k in sorted(file_lst.keys())]

    def get_only_specified_data(self, dat_items, all_data):
        ret = {}
        # 二次过滤，每个字段最多返回MAX_RESULT_DATA_LEN个点数据
        step = 1
        if len(all_data) > MAX_RESULT_DATA_LEN:
            step = int(len(all_data) / MAX_RESULT_DATA_LEN)
        for index in range(0, len(all_data), step):
            block_dat = all_data[index]
            timestamp = int(block_dat["timestamp"])
            for key in dat_items:
                module, field = key.split(".")
                if module not in block_dat.keys() or field not in block_dat[module].keys():
                    break
                if field not in ret.keys():
                    ret[field] = []
                ret[field].append({"x": timestamp, "y": block_dat[module][field]})
        return ret

    @staticmethod
    def parse_block_len(in_file):
        buf = in_file.read(2)
        if not buf:
            return 0
        high_byte, low_byte = struct.unpack('2B', buf)
        return (high_byte << 8) + low_byte

# mon = MonitorParser()
# dat = mon.parse_json_file('./temp/m_1460480408.json')
# strxx = "systemHardware.cpuUsage,systemHardware.memoryUsage"
# mon.get_only_specified_data(strxx.split(","), dat)
# print dat
# mon.parse_from_to(1560993152, 1560994952)
