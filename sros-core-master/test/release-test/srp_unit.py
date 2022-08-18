#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @File: srp_unit.py
# @Author: pengjiali
# @Date: 19-11-19
# @Copyright: Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# @Describe:

import sys

sys.path.append('../communication')
from async_result import AsyncResult, AsyncResultState
import asyncio
import path
from srp import SRP
import time
import threading
import logging

FORMAT = ('%(asctime)-15s %(threadName)-15s'
          ' %(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
logging.basicConfig(stream=sys.stdout, level=logging.INFO, format=FORMAT)


class SrpUnit(object):
    _instance = None
    _init_flag = False

    def __init__(self):
        if SrpUnit._init_flag:
            return
        SrpUnit._init_flag = True

        self.srp = SRP()
        self.srp.set_system_state_callback(self._sys_state_callback)
        self.srp.login('192.168.83.201', 'admin', 'admin')
        self.loop = asyncio.new_event_loop()
        self._async_result = AsyncResult(self.loop)

        self._backup_configs = {}  # 备份的参数

        def start_loop(loop):
            asyncio.set_event_loop(loop)
            loop.run_forever()

        t = threading.Thread(target=start_loop, args=(self.loop,))
        t.start()

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = object.__new__(cls)
        return cls._instance

    def _sys_state_callback(self, data):
        # print(data)
        pass

    def stop(self):
        def func(loop):
            loop.stop()

        self.loop.call_soon_threadsafe(func, self.loop)

    def backup_config(self, key):
        config = self.srp.get_config(key)
        value = config['value']

        is_contain = False
        for backup_key in self._backup_configs.keys():
            if backup_key == key:
                self._backup_configs[key] = value
                is_contain = True
                break
        if not is_contain:
            self._backup_configs[key] = value

    def restore_config(self, key):
        is_contain = False
        for backup_key, backup_value in self._backup_configs.items():
            if backup_key == key:
                self.srp.set_config({backup_key: backup_value})
                is_contain = True
                break
        if not is_contain:
            raise Exception('Config ' + key + ' not backup')


srp_unit = SrpUnit()
srp = srp_unit.srp

if __name__ == '__main__':
    srp_unit1 = SrpUnit()
    srp_unit2 = SrpUnit()
    print(srp_unit1, srp_unit2)
