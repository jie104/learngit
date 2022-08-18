#!/usr/bin/env python
# -*- coding:utf-8 -*-
# file sros_protobuf_test.py
# author pengjiali
# date 19-6-26.
# copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# describe

import sys

sys.path.append('./communication')
from async_result import AsyncResult, AsyncResultState
import asyncio
import path
from srp import SRP, Pose
import time
import threading
import ctypes
import binascii
import logging


class SrosProtobufTest:
    def __init__(self):
        self._srp = SRP()
        self._srp.set_system_state_callback(self._sys_state_callback)
        self._srp.set_hardware_state_callback(self._hardware_state_callback)
        self._srp.set_notify_action_task_finished_callback(
            self._notify_action_task_finished_callback)
        self._srp.login('127.0.0.1', 'admin', 'admin')
        self.loop = asyncio.new_event_loop()
        self._async_result = AsyncResult(self.loop)

        def start_loop(loop):
            asyncio.set_event_loop(loop)
            loop.run_forever()

        t = threading.Thread(target=start_loop, args=(self.loop,))
        t.start()

    def __del__(self):
        print('del')
        self.loop.stop()

    def map_switching(self):
        self._srp.map_switching("FMS车型组测试A", 66)
        # self._srp.map_switching("FMS车型组测试A", 0, locate_pose=Pose(-27260, 9790, 4730), is_force_locate=True)

    def move_to_station_test(self):
        self._srp.move_to_station(333, 999)

    def move_to_stations_test(self):
        self._srp.move_to_station(333, 0, station_ids=[117, 94])

    def movemet_follow_path_test(self):
        paths = []

        paths.append(path.Path.create_rotate_path(3000))
        paths.append(path.Path.create_line_path(0, 0, -8750, 1267))
        paths.append(path.Path.create_rotate_path(0))
        p = path.Path.create_line_path(-8750, 1267, 1267, 1267)
        # p.limit_v = 200
        # p = path.Path.create_rotate_path(3140)
        # p.limit_w = 20
        paths.append(p)

        self._srp.move_follow_path(66, paths)

    def movement_follow_path_cancel_test(self):
        paths = []
        line_path = path.Path.create_line_path(6854, -1150, 656, -1144)
        paths.append(line_path)
        self._srp.move_follow_path(66, paths)
        time.sleep(3)
        self._srp.cancel_movement_task()

    def movement_follow_path_args_check(self):
        paths = []
        paths.append(path.Path.create_rotate_path(1570))
        paths.append(path.Path.create_line_path(350, -5950, 350, -1960))
        paths.append(path.Path.create_rotate_path(3142))

        self._srp.move_follow_path(66, paths)

    def action_cancel_test(self):
        self._srp.excute_action_task(77, 129, 10, 0)
        time.sleep(3)
        self._srp.cancel_action_task()

    def action_param3_test(self):
        self._srp.excute_action_task(77, 40, 10, 0, 0)
        time.sleep(3)

    def violence_continuous_action_test(self):
        '''
        暴力连续发动作，测试是否启动唯一的action task
        :return: None
        '''
        for i in range(1, 1000):
            self._srp.async_excute_action_task(i, 129, 10, 0)
        pass

    def read_input_registers_test(self):
        ret = self._srp.read_input_registers(30151, 1)
        print(ret)
        print(int.from_bytes(ret, 'litter'))
        # first_value = ret[0]
        # vale = ctypes.c_int16(first_value)
        # print(vale.value)

    async def continuous_action_test(self):
        '''
        连续发送动作（发送动作，等待动作的Notification）
        :return: None
        '''
        for i in range(1, 10):
            self._async_result.clear()
            self._srp.excute_action_task(i, 129, 1, 0)
            print(i, 129, 1, 0)
            await self._async_result.wait()

    def set_current_map_test(self):
        self._srp.set_current_map('20190412')

    def checkpoint_test(self):
        paths = []
        paths.append(path.Path.create_rotate_path(0))
        paths.append(path.Path.create_line_path(-8720, 1250, 1080, 1250))
        paths.append(path.Path.create_rotate_path(3140))
        paths.append(path.Path.create_line_path(1080, 1250, -8720, 1250))
        paths.append(path.Path.create_rotate_path(0))


        self._srp.move_follow_path(66, paths, checkpint_no=-1)
        print(999)

    def set_checkpoint_test(self):
        self._srp.set_checkpoint(8)

    def request_all_state_test(self):
        state = self._srp.request_all_state()
        print(state)

    def request_once_system_state_test(self):
        self._srp.request_system_state()

    def request_once_hardware_state_test(self):
        self._srp.request_hardware_state()

    def _sys_state_callback(self, data):
        try :
            # print(data.faults)
            pass
        except:
            pass
        pass

    def _hardware_state_callback(self, data):
        try :
            # print(data)
            pass
        except:
            pass
        pass

    def _notify_action_task_finished_callback(self, action_task):
        self._async_result.accept(action_task)

    def set_auto_upload_laser_point_test(self):
        self._srp.set_auto_upload_laser_point(True)



if __name__ == '__main__':
    FORMAT = ('%(asctime)-15s %(threadName)-15s'
              ' %(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
    logging.basicConfig(stream=sys.stdout, level=logging.DEBUG, format=FORMAT)
    test = SrosProtobufTest()
    # test.read_input_registers_test()
    # test.loop.call_soon_threadsafe(test.request_all_state_test)
    # time.sleep(2)
    # test.loop.call_soon_threadsafe(test.move_to_stations_test)
    # test.loop.call_soon_threadsafe(test.request_once_system_state_test)
    # test.loop.call_soon_threadsafe(test.request_once_hardware_state_test)
    # time.sleep(3)
    # test.loop.call_soon_threadsafe(test.map_switching)
    # test.loop.call_soon_threadsafe(test.checkpoint_test)
    time.sleep(2)
    test.loop.call_soon_threadsafe(test.movemet_follow_path_test)
    # test.loop.call_soon_threadsafe(test.set_auto_upload_laser_point_test)
    # test.loop.call_soon_threadsafe(test.set_checkpoint_test)
    # test.loop.call_soon_threadsafe(test.set_current_map_test)
    # test.loop.call_soon_threadsafe(test.move_to_station_test())
    # test.loop.call_soon_threadsafe(test.movement_follow_path_args_check)
    # test.loop.call_soon_threadsafe(test.action_cancel_test)
    test.loop.call_soon_threadsafe(test.action_param3_test)
    # test.loop.call_soon_threadsafe(test.violence_continuous_action_test)
    # asyncio.run_coroutine_threadsafe(test.continuous_action_test(), test.loop).result(20)
