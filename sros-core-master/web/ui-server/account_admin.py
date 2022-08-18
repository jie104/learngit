# -*- coding: utf-8 -*-

import json
import web
import hashlib

from database import AccountAdmin
from database import UserMissionAdmin

account_adm = AccountAdmin()
ADMIN_PERMISSION_MIN = 20

class API_AccountAdmin:
    def __init__(self):
        return

    # 获取所有用户
    def GET(self):
        user_id = int(web.input(id=2).id)
        user = account_adm.get(user_id)
        if not user:
            raise web.web.notfound()
        permission = int(user['permission'])
        accounts = []
        for account in account_adm.get_all():
            if int(account['permission']) <= permission:
                accounts.append(account)
        return json.dumps(accounts)


class API_Account:
    def GET(self, input_param):
        input_type = web.input(type='').type
        input_param = input_param.encode('utf-8')
        # 没有指定（兼容老接口）或指定为username, input_param为用户名
        if input_type == '' or input_type == 'username':
            user_id = account_adm.get_id_by_name(input_param)
            account = account_adm.get(int(user_id))
            return json.dumps(account)
        elif input_type == 'pincode': # 指定pin码
            account = account_adm.get_accout_by_pincode(hashlib.md5(input_param).hexdigest())
            return json.dumps(account)

    def POST(self, user_id):
        # 判断用户是否有添加，修改账户的权限
        user_id = int(user_id)
        if int(account_adm.get(user_id)['permission']) < ADMIN_PERMISSION_MIN:
            raise web.NotAcceptable('Permission Denied')
            return

        req_str = web.data().encode('utf-8')
        json_dat = json.loads(req_str)
        password = json_dat['passwd'].encode('utf-8')
        pincode = json_dat['pincode']
        # 只有普通账号权限才支持设置pin码
        if int(json_dat['permission']) >= ADMIN_PERMISSION_MIN:
            pincode = ""
        mission_ids = json_dat['mission_ids']
        # id <= 0添加新用户,否则更新用户数据
        if int(json_dat['id']) <= 0:
            ret = account_adm.add(json_dat['username'], hashlib.md5(password).hexdigest(), json_dat['permission'],
                                  hashlib.md5(pincode).hexdigest())
            json_dat['id'] = ret
        else:
            ret = account_adm.update(json_dat['id'], json_dat['username'], json_dat['permission'])
            if password:
                ret = account_adm.update_password(json_dat['id'], hashlib.md5(password).hexdigest())
            if pincode and int(json_dat['permission']) < ADMIN_PERMISSION_MIN:
                ret = account_adm.update_pincode(json_dat['id'], hashlib.md5(pincode).hexdigest())
        if int(json_dat['permission']) < ADMIN_PERMISSION_MIN:
            UserMissionAdmin.update_mission(json_dat['id'], mission_ids)
        return json.dumps({'result': ret})

    def DELETE(self, user_id):
        user_id = int(user_id)
        if user_id <= 3:
            raise web.NotAcceptable("Not Support Operation")
            return json.dumps({'result': 2})
        ret = account_adm.delete(user_id)
        UserMissionAdmin.delete_mission_by_user(user_id)
        return json.dumps({'result': ret})

    # 如果不重写，post会报错
    def OPTIONS(self, user_id):
        pass
