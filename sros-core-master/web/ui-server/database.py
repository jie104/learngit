# -*- coding: utf-8 -*-

import web
import json
import time

from sros_log import _logger

# 数据库连接配置，host=数据库服务器地址，user=用户名，pw=密码，db=数据库名称
db_main = web.database(dbn='sqlite', db="/sros/db/main.db3")
table_name_user = 'login'
table_name_mission = 'mission'
table_name_task_tpl = 'tpl_task'
table_name_config = 'config'
table_name_user_missions = 'user_missions'
table_name_schedule = 'schedule'
table_name_fixed_record = 'fixed_record'
table_name_fixed_device = 'fixed_device'
table_name_upgrade_record = 'upgrade_record'
table_name_mission_record = 'mission_record'
table_name_maps = 'maps'

MAX_RECORD_LIMIT = 100

system_tpls = ['charge']


class MapsAdmin:
    def __init__(self):
        return

    @staticmethod
    def has_map(name):
        rs = db_main.select(table_name_maps, where="name = '%s'" % name)
        if rs:
            return True
        return False

    @staticmethod
    def get_all_maps():
        try:
            rs = db_main.select(table_name_maps, order='top_timestamp, modify_time DESC')
            if rs:
                return list(rs)
            return []
        except BaseException as e:
            print(e)
        return []

    @staticmethod
    def get_map_info_by_name(name):
        rs = db_main.select(table_name_maps, where="name = '%s'" % name)
        maps = list(rs)
        assert len(maps) < 2
        if maps:
            return maps[0]
        return None

    @staticmethod
    def create_map(map):
        try:
            map_id = db_main.insert(
                table_name_maps,
                name=map["name"], md5=map["md5"], modify_time=map["modify_time"],
                create_timestamp=int(time.time()), top_timestamp=0,
            )
        except Exception as e:
            _logger.error(e)
            return None
        return map_id

    @staticmethod
    def update_map_by_name(name, map_info):
        db_main.update(table_name_maps, "name = '%s'" % name, md5=map_info["md5"], modify_time=map_info["modify_time"])

    @staticmethod
    def update_map_name_by_name(name, new_name):
        db_main.update(table_name_maps, "name = '%s'" % name, name=new_name)

    @staticmethod
    def update_map_top_timestamp__by_name(name, top_timestamp):
        db_main.update(table_name_maps, "name = '%s'" % name, top_timestamp=top_timestamp)

    @staticmethod
    def delete_map_by_name(name):
        db_main.delete(table_name_maps, where="name = '%s'" % name)


class ConfigAdmin:
    def __init__(self):
        return

    @staticmethod
    def get_config(key):
        ret = list(db_main.select(table_name_config, where="key = '%s'" % key))
        if not ret:
            return None
        return ret[0]

    # 获取模板
    @staticmethod
    def get_config_by_id(config_id):
        rs = db_main.select(table_name_config, where='id = %d' % int(config_id))
        configs = list(rs)
        assert len(configs) < 2
        if configs:
            return configs[0]
        return None

    @staticmethod
    def get_all_configs():
        try:
            rs = db_main.select(table_name_config)
            if rs:
                return list(rs)
            return []
        except BaseException as e:
            print(e)
        return []

    @staticmethod
    def create_config(config):
        try:
            config_id = db_main.insert(
                table_name_config,
                id=int(config["id"]),
                key=config["key"],
                name=config["name"],
                value=config["value"],
                default_value=config["default_value"],
                value_units=config["value_units"],
                value_type=config["value_type"],
                value_range=config["value_range"],
                description=config["description"],
                permission=config["permission"],
                changed_time=int(time.time()),
                changed_user=config["changed_user"],
                is_valid=config["is_valid"]
            )
        except Exception as e:
            print(e)
            return None
        print(config_id)
        return config_id

    @staticmethod
    def update_config(is_update, config_old, config_new):
        is_valid = config_old["is_valid"]
        if not is_update:
            is_valid = 0
        db_main.update(
            table_name_config,
            'id = %d' % int(config_old["id"]),
            key=config_old["key"],
            name=config_old["name"],
            value=config_new["value"],
            default_value=config_old["default_value"],
            value_units=config_old['value_units'],
            value_type=config_old["value_type"],
            value_range=config_old["value_range"],
            description=config_old["description"],
            permission=config_old["permission"],
            changed_time=int(time.time()),
            changed_user=config_old["changed_user"],
            is_valid=is_valid
        )

    # @staticmethod
    # set is_valid = 0
    # # def delete_config(config_id):
    #     db_main.update(table_name_config, where='id = %d' % int(config_id))


class AccountAdmin:
    def __init__(self):
        return

    def get_all(self):
        return list(db_main.select(table_name_user))

    @staticmethod
    def get(user_id):
        accounts = list(db_main.select(table_name_user, where='id = %d' % user_id))
        assert len(accounts) == 1
        return accounts[0]

    def add(self, name, password, permission, pincode):
        # 账号名称不能相同
        rs = db_main.select(table_name_user, where='username = "%s"' % name)
        if rs:
            return 0
        ret = db_main.insert(table_name_user, session_id=0, username=name, passwd=password, permission=permission,
                             login_time=0, item_valid=1, pincode=pincode)
        return int(ret)

    def update(self, user_id, name, permission):
        if not self.has_account(user_id):
            return 0
        db_main.update(table_name_user, where='id = %d' % user_id, username=name, permission=permission)
        return 1

    def update_password(self, user_id, password):
        if not self.has_account(user_id):
            return 0
        db_main.update(table_name_user, where='id = %d' % user_id, passwd=password)
        return 1

    def update_pincode(self, user_id, pincode):
        if not self.has_account(user_id):
            return 0
        db_main.update(table_name_user, where='id = %d' % user_id, pincode=pincode)
        return 1

    def delete(self, user_id):
        if not self.has_account(user_id):
            return 0
        db_main.delete(table_name_user, where='id = %d' % user_id)
        return 1

    def has_account(self, user_id):
        rs = db_main.select(table_name_user, where='id = %d' % user_id)
        if rs:
            return True
        return False

    def get_id_by_name(self, username):
        rs = db_main.select(table_name_user, where='username = "%s"' % username)
        if not rs:
            return -1
        return list(rs)[0].id

    def get_accout_by_pincode(self, pincode):
        rs = db_main.select(table_name_user, where='pincode = "%s"' % pincode)
        if not rs:
            return None
        accounts = list(rs)
        assert len(accounts) == 1
        return accounts[0]


class TaskTemplateAdmin:
    @staticmethod
    def is_system_template(name):
        for tpl in system_tpls:
            if tpl == name:
                return True
        return False

    # 新增模板
    @staticmethod
    def create_template(template):
        try:
            tpl_id = db_main.insert(table_name_task_tpl, name=template["name"], desc=template["desc"],
                                    owner_id=template["owner_id"],
                                    create_time=int(time.time()), last_modified_time=int(time.time()),
                                    body=template['body'])
        except Exception as e:
            print(e)
            return None
        return tpl_id

    # 获取模板
    @staticmethod
    def get_template(tpl_id):
        rs = db_main.select(table_name_task_tpl, where='id = %d' % int(tpl_id))
        tpls = list(rs)
        assert len(tpls) < 2
        if tpls:
            tpl = tpls[0]
            tpl["body"] = json.loads(tpl["body"])
            return tpl
        return None

    # 根据名称获取模板
    @staticmethod
    def get_template_by_name(tpl_name):
        try:
            rs = db_main.select(table_name_task_tpl, where='name = "%s"' % tpl_name)
        except BaseException as e:
            print(e)
            return 1
        tpls = list(rs)
        assert len(tpls) < 2
        if tpls:
            return tpls[0]
        return None

    # 更新模板
    @staticmethod
    def update_template(template):
        db_main.update(table_name_task_tpl, 'id = %d' % template['id'], name=template['name'], desc=template["desc"],
                       last_modified_time=int(time.time()), body=template['body'])

    # 删除模板
    @staticmethod
    def delete_template(tpl_id):
        db_main.delete(table_name_task_tpl, where='id = %d' % tpl_id)

    # 获取所有任务模板
    @staticmethod
    def get_all_templates():
        rs = db_main.select(table_name_task_tpl)
        tpls = list(rs)
        for tpl in tpls:
            tpl["body"] = json.loads(tpl["body"])
        return tpls


class UserMissionAdmin:
    # 新增mission
    @staticmethod
    def create_mission(mission):
        rs = db_main.insert(table_name_mission, owner_id=mission["owner_id"], name=mission['name'],
                            map_name=mission['mapName'],
                            mission_desc=mission['mission_desc'], top_timestamp=mission['top_timestamp'],
                            create_time=int(time.time()), last_modified_time=int(time.time()), body=mission['body'])
        return rs

    # 获取mission详细信息
    @staticmethod
    def get_detailed_mission(mission_id):
        rs = db_main.select(table_name_mission, where='id = %d AND item_valid = 1' % int(mission_id))
        missions = list(rs)
        assert len(missions) < 2
        if missions:
            return missions[0]
        return None

    # 根据任务名称和地图名称获取任务
    @staticmethod
    def get_detailed_mission_by_name(name, map_name):
        rs = db_main.select(table_name_mission,
                            where='name = "%s" AND map_name = "%s" AND item_valid = 1' % (name, map_name))
        missions = list(rs)
        assert len(missions) < 2
        if missions:
            return missions[0]
        return None

    # 更新mission详细信息，用于编辑mission
    @staticmethod
    def update_detailed_mission(mission):
        db_main.update(table_name_mission, 'id = %d' % mission['id'], name=mission['name'], map_name=mission['mapName'],
                       mission_desc=mission['mission_desc'], last_modified_time=int(time.time()),
                       top_timestamp=mission['top_timestamp'], body=json.dumps(mission['body']))

    # 用户任务包括两部分，自己创建的任务+分配给其的任务
    @staticmethod
    def get_missions(user_id):
        mission_ids = UserMissionAdmin.get_user_mission_ids(user_id)
        if user_id > 0:
            permission = int(AccountAdmin.get(user_id)['permission'])
        else:
            permission = 10
        rs = db_main.select(table_name_mission, where='item_valid = 1')
        missions = []
        for r in rs:
            if user_id == 0 or r.owner_id == user_id or r.id in mission_ids or permission >= 20:
                r['body'] = json.loads(r['body'])
                missions.append(r)
        return missions

    # 允许指定用户访问指定mission
    @staticmethod
    def add_mission(user_id, mission_id):
        db_main.insert(table_name_user_missions, user_id=user_id, mission_id=mission_id)

    # 删除指定mission
    @staticmethod
    def delete_mission(mission_id):
        db_main.update(table_name_mission, 'id = %d' % mission_id, item_valid=0)
        db_main.delete(table_name_user_missions, where='mission_id = %d' % mission_id)

    @staticmethod
    def restore_mission(mission_id):
        db_main.update(table_name_mission, 'id = %d' % mission_id, item_valid=1)

    # 解除指定用户的mission访问关系
    @staticmethod
    def delete_mission_by_user(user_id):
        db_main.delete(table_name_user_missions, where='user_id = %d' % user_id)

    # 更新指定用户允许访问的mission, 用于重新指派用户允许操作的mission
    @staticmethod
    def update_mission(user_id, mission_ids):
        UserMissionAdmin.delete_mission_by_user(user_id)
        for mission_id in mission_ids:
            UserMissionAdmin.add_mission(user_id, mission_id)

    # 获取被分配的任务ids
    @staticmethod
    def get_user_mission_ids(user_id):
        rs = db_main.select(table_name_user_missions, what='mission_id', where='user_id = %d' % user_id)
        ids = []
        for r in rs:
            ids.append(r['mission_id'])
        return ids


class ScheduleAdmin:
    @staticmethod
    def create(dat):
        try:
            rs = db_main.insert(table_name_schedule,
                                name=dat["name"],
                                detail=dat['desc'],
                                timestamp=int(time.time() * 1000),
                                type=dat['type'],
                                is_enable=dat['isEnable'],
                                task_id=dat['taskId'],
                                dat=json.dumps(dat['dat'])
                                )
            return rs
        except BaseException as e:
            print(e)
        return 0

    @staticmethod
    def get_all_schedules():
        try:
            rs = db_main.select(table_name_schedule)
            if rs:
                return list(rs)
            return []
        except BaseException as e:
            print(e)
        return []

    @staticmethod
    def update(dat):
        try:
            db_main.update(table_name_schedule,
                           'id = %d' % dat['id'],
                           name=dat["name"],
                           detail=dat['desc'],
                           timestamp=int(time.time() * 1000),
                           type=dat['type'],
                           is_enable=dat['isEnable'],
                           task_id=dat['taskId'],
                           dat=json.dumps(dat['dat'])
                           )
        except BaseException as e:
            print(e)
            return False
        return True

    @staticmethod
    def delete(schedule_id):
        try:
            db_main.delete(table_name_schedule, where='id = %d' % schedule_id)
            return True
        except BaseException as e:
            print(e)
        return False


class FixedRecord:
    @staticmethod
    def get_all_records():
        try:
            rs = db_main.select(table_name_fixed_record, limit=MAX_RECORD_LIMIT, order='timestamp desc')
            record_list = list(rs)
            for record in record_list:
                uid = record["id"]
                record["devices"] = FixedDevice.get_devices(uid)
            return record_list
        except BaseException as e:
            print(e)
            raise web.NotFound("failed get fixed records")
        return []

    @staticmethod
    def create(dat):
        try:
            rs = db_main.insert(table_name_fixed_record,
                                robot_sn=dat["robotSN"],
                                username=dat["username"],
                                remark=dat['remark'],
                                mileage=dat['mileage'],
                                poweron_number=dat['powerCycle'],
                                timestamp=dat['timestamp'],
                                creator=dat['creator']
                                )
            for device in dat["devices"]:
                FixedDevice.create(rs, device)
            return rs
        except BaseException as e:
            print(e)
            raise web.NotFound("failed create fixed record")
        return 0


class FixedDevice:
    @staticmethod
    def get_devices(record_id):
        try:
            rs = db_main.select(table_name_fixed_device, where='record_id = %d' % record_id)
            if rs:
                return list(rs)
        except BaseException as e:
            print(e)
        return []

    @staticmethod
    def create(record_id, dat):
        try:
            rs = db_main.insert(table_name_fixed_device,
                                record_id=record_id,
                                category=dat['category'],
                                name=dat['name'],
                                action=dat['action']
                                )
            return rs
        except BaseException as e:
            print(e)
        return 0


class UpgradeRecord:
    @staticmethod
    def get_all_records(type_code):
        try:
            rs = db_main.select(table_name_upgrade_record, where='type_code = %d' % type_code,
                                limit=MAX_RECORD_LIMIT, order='start_time desc')
            if rs:
                return list(rs)
        except BaseException as e:
            print(e)
        return []


class MissionRecord:

    @staticmethod
    def set_all_clear():
        try:
            rs = db_main.delete(table_name_mission_record, where='1 = 1')
            print rs
        except BaseException as e:
            print(e)
        return 0
