#!/usr/bin/env python
# -*- coding:utf-8 -*-
# file srp.py
# author pengjiali
# date 19-6-19.
# copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# describe 与sros进行protobuf通信的封装

import asyncio
import threading
import srp_protobuf
from srp_protobuf import Pose
import main_pb2
from async_result import AsyncResult, AsyncResultState
import socket
import logging
import time


class SrpProtocol(asyncio.Protocol):
    def __init__(self, srp):
        self._srp = srp

    def connection_made(self, transport):
        self._srp.connection_made(transport)

    def data_received(self, data):
        self._srp.data_received(data)

    def error_received(self, exc):
        self._srp.error_received(exc)

    def connection_lost(self, exc):
        self._srp.connection_lost(exc)


class SRP:
    '''
    @describe:  本类用来和sros通信
                本类内部有一个线程用于处理protobuf协议的解析、收发，本类是线程安全的
                本类的方法既支持同步通信的方式也支持异步通信的方式，同步通信的方式主要用来测试，比如连续发送task而不等回复
                默认为异步函数，同步函数前面都加了前缀:sync_,和node.js的库命名规范类似
    @NOTE: 异步调用时，假定不会同时调用， 现在只有一个AsyncResult类来阻塞等待sros的响应
    '''

    def __init__(self):
        self._loop = asyncio.new_event_loop()
        self._transport = None
        self._protocl = None
        self._protobuf = srp_protobuf.SrpProtobuf(self.write_callback, self.response_callback)
        self._seq = 0
        self._wait_seq = 0  # 等待回复的seq
        self._async_result = AsyncResult(self._loop)

        def start_loop(loop):
            asyncio.set_event_loop(loop)
            loop.run_forever()

        t = threading.Thread(target=start_loop, args=(self._loop,))
        t.start()

    def set_system_state_callback(self, fun):
        self._protobuf.set_system_state_callback(fun)

    def set_hardware_state_callback(self, fun):
        self._protobuf.set_hardware_state_callback(fun)

    def set_laser_point_callback(self, fun):
        self._protobuf.set_laser_point_callback(fun)

    def set_notify_move_task_finished_callback(self, fun):
        self._protobuf.set_notify_move_task_finished_callback(fun)

    def set_notify_action_task_finished_callback(self, fun):
        self._protobuf.set_notify_action_task_finished_callback(fun)

    def set_notify_mission_list_change_callback(self, fun):
        self._protobuf.set_notify_mission_list_change_callback(fun)

    def login(self, ip_addr, user_name, passwd, session_id=0):
        '''
        登录sros
        :param ip_addr: target sros's ip address
        :param user_name: clear text
        :param passwd: clear text
        :param session_id: 重连时，可以带session_id
        :return: none
        '''
        f = asyncio.run_coroutine_threadsafe(self._connect(ip_addr), self._loop)
        result = f.result(3)
        if not result:
            return False
        print('Connect succeed ip:', ip_addr, 'port:', 5001)

        self._run_async_threadsafe(self._protobuf.login, user_name, passwd, session_id)

    def request_all_state(self):
        return self._run_async_threadsafe(self._protobuf.request_all_state)

    def request_system_state(self):
        return self._run_async_threadsafe(self._protobuf.request_system_state)

    def request_hardware_state(self):
        return self._run_async_threadsafe(self._protobuf.request_hardware_state)

    # 一下为异步指令
    def triger_emergency(self):
        self._run_sync_threadsafe(self._protobuf.setEmergency)

    def triger_usersetstate(self):
        self._run_sync_threadsafe(self._protobuf.set_user_set_state)

    def cancel_emergency(self):
        self._run_sync_threadsafe(self._protobuf.cancelEmergencyState)

    def map_switching(self, map_name, locate_station_id, locate_pose=Pose(0, 0, 0), is_force_locate=False):
        self._run_sync_threadsafe(self._protobuf.map_switching, map_name, locate_station_id, locate_pose,
                                  is_force_locate)

    def pause_movement(self, level):
        self._run_sync_threadsafe(self._protobuf.pause_movement, level)

    def continue_movement(self):
        self._run_async_threadsafe(self._protobuf.continue_movement)

    def set_auto_upload_laser_point(self, enable):
        self._run_async_threadsafe(self._protobuf.set_auto_upload_laser_point, enable)


    def move_to_station(self, no, station_id, avoid_policy=main_pb2.MovementTask.OBSTACLE_AVOID_WAIT, station_ids=[]):
        self._run_sync_threadsafe(self._protobuf.move_to_station, no, station_id, avoid_policy, station_ids)

    def move_follow_path(self, no, paths, avoid_policy=main_pb2.MovementTask.OBSTACLE_AVOID_WAIT, checkpint_no=0):
        self._run_sync_threadsafe(self._protobuf.move_follow_path, no, paths, avoid_policy, checkpint_no)

    def set_current_map(self, map_name):
        self._run_async_threadsafe(self._protobuf.set_current_map, map_name)

    def set_checkpoint(self, checkpint_no):
        self._run_sync_threadsafe(self._protobuf.set_checkpoint, checkpint_no)

    def cancel_movement_task(self):
        self._run_sync_threadsafe(self._protobuf.cancel_movement_task)

    def excute_action_task(self, no, action_id, param0, param1, param2=0):
        self._run_sync_threadsafe(self._protobuf.excute_action_task, no, action_id, param0, param1, param2)

    def cancel_action_task(self):
        self._run_sync_threadsafe(self._protobuf.cancel_action_task)

    def read_input_registers(self, start_addr, count):
        return self._run_sync_threadsafe(self._protobuf.readInputRegisters, start_addr, count)

    def get_config(self, key=''):
        configs = self._run_sync_threadsafe(self._protobuf.get_config)
        if key != '':
            for config in configs:
                if config['key'] == key:
                    return config
        else:
            return configs

    def set_config(self, configs):
        self._run_sync_threadsafe(self._protobuf.set_config, configs)

    # 一下为异步指令
    def async_move_to_station(self, no, station_id, avoid_policy=main_pb2.MovementTask.OBSTACLE_AVOID_WAIT):
        self._run_async_threadsafe(self._protobuf.move_to_station, no, station_id, avoid_policy)

    def async_excute_action_task(self, no, action_id, param0, param1, param2=0):
        self._run_async_threadsafe(self._protobuf.excute_action_task, no, action_id, param0, param1, param2)

    def _run_sync_threadsafe(self, fun, *args, timeout=3):
        f = asyncio.run_coroutine_threadsafe(self._async_request(fun, *args), self._loop)
        return f.result(timeout)

    async def _async_request(self, fun, *args):
        self._seq += 1
        self._wait_seq = self._seq
        self._async_result.clear()
        fun(self._seq, *args)
        return await self._async_result.wait()

    def _run_async_threadsafe(self, fun, *args):
        self._loop.call_soon_threadsafe(self._syncRequest, fun, *args)

    def _syncRequest(self, fun, *args):
        self._seq += 1
        self._wait_seq = self._seq
        fun(self._seq, *args)

    async def _connect(self, ip_addr):
        '''
        由于同样的ip和port链接sros经常出问题，所以此处一直用端口号为8888的端口号链接sros
        :param ip_addr:
        :return:
        '''
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # SOURCE_PORT = 8888
            # sock.bind(('0.0.0.0', SOURCE_PORT))
            sock.connect((ip_addr, 5001))
            self._transport, self._protocl = await self._loop.create_connection(lambda: SrpProtocol(self), sock=sock)
            return True
        except BaseException as e:
            print(e)
        return False

    def connection_made(self, transport):
        print('connected!')

    def data_received(self, data):
        self._protobuf.onRead(data)

    def error_received(self, exc):
        print('Error received:', exc)

    def connection_lost(self, exc):
        print("Socket closed", exc)

    def write_callback(self, data):
        self._transport.write(data)

    def response_callback(self, seq, response_type, ok, value=None, result_code=None):
        if seq == self._wait_seq:
            if ok:
                self._async_result.accept(value)
            else:
                self._async_result.reject(result_code)


if __name__ == '__main__':
    srp = SRP()
    try:
        #srp.login('192.168.83.201', 'admin', 'admin')
        #srp.triger_emergency()
        #srp.move_to_station(1, 1)
        srp.login('192.168.71.45', 'admin', 'admin')
        time.sleep(1)
        srp.triger_usersetstate()
    except BaseException as e:
        print(e)

    #for i in range(2, 50):
    #    srp.async_move_to_station(i, 1)
