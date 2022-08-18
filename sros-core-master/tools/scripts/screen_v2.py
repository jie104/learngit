#!/usr/bin/env python
# -*- coding:utf-8 -*-
# file screen_v2.py
# date 19-10-16.
# copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# describe

"""
fms api 寄存器地址分配：
（由于这些地址以后都可能会被废弃，所以都定义以9001开始）

离散量输入状态：
19006 ： 车辆上下线状态（1：上线； 0：下线）

线圈状态：
09001 ： 暂停车辆当前订单
09002 ： 继续车辆当前订单
09003 ： 取消车辆当前订单
09004 : 跳过当前子任务继续执行
09005 : 从当前子任务继续执行
09006 : 1 - 将车辆上线， 0 - 将车辆下线

09007 : 清除路径占用
09008 : 返回充电
09009 : 返回待命点

输入寄存器：
39001 ： 车辆当前订单ID  (int32_t)
39003 ： 车辆当前订单状态，订单状态如下： (uint16_t)
    0 NONE： 无
    1 QUEUEING : 正在排队
    2 EXECUTING: 正在执行
    3 PAUSED: 正处于暂停状态
    4 SUCCESS: 订单被成功执行完
    5 FAILED: 订单未能成功执行完
    6 CANCELLED: 订单被取消
    7 DELETED: 订单被删除

39004 ： 订单进度（%） (uint16_t)
...

39017 ： 当前车辆正在排队的订单ID 1 (int32_t)
39019 ： 当前车辆正在排队的订单ID 2 (int32_t)
39021 ： 当前车辆正在排队的订单ID 3 (int32_t)
39023 ： 当前车辆正在排队的订单ID 4 (int32_t)
39025 ： 当前车辆正在排队的订单ID 5 (int32_t)

保持寄存器：
49001 ： 给当前车辆启动订单模板，输入值为订单模板ID (int32_t)
49003 : 取消订单模板（int32_t)

"""
import os
def checkChips():
    strTk1="tk1"
    strNxp="nxp"
    cmd = ('echo $(hostname) | grep ' + strTk1)
    rs = os.system(cmd)
    if rs == 256:
        print("system is",strNxp)
        return True
    print("system is",strTk1)
    return False

if checkChips() :
    import requests
    from requests.auth import HTTPDigestAuth
    from lxml import etree


from pymodbus.interfaces import IModbusSlaveContext
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext, ModbusSequentialDataBlock
from pymodbus.server.sync import StartTcpServer, StartSerialServer
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.exceptions import NotImplementedException
from pymodbus.transaction import ModbusRtuFramer
import urllib.request as request
import urllib.parse
import sqlite3
import json
import time
import threading

import logging
from enum import Enum
import sros_log
import copy
import logging

_logger = logging.getLogger(__name__)


class OrderState(Enum):
    QUEUEING = 1
    CANCELLED = 2
    EXECUTING = 3
    FAILED = 4
    SUCCESS = 5
    DELETED = 6
    HELD = 7
    REJECTED = 8
    HANG = 9
    QUEUED = 10
    QUEUEING_HELD = 11


class ScreenSlaveContext(IModbusSlaveContext):
    ''' TODO
    This creates a modbus data model that connects to
    a remote device (depending on the client used)
    '''

    def __init__(self, client, fms_api, get_fms_state_fps, unit=None):
        ''' Initializes the datastores

        :param client: The client to retrieve values with
        :param unit: Unit ID of the remote slave
        :param fms_api: fms api
        :param get_fms_state_fps: 获取fms系统状态频率
        '''
        _logger.info('__init__')

        self._client = client
        self.unit = unit
        self.__build_mapping()
        self.__fms_api = fms_api
        self.__cur_vehicle_cur_excute_order_id = 0
        self.__vehicle_queueing_order_list = []  # 存储排队的列表
        self.__get_fms_state_fps = get_fms_state_fps
        self.hfms = False
        self._lock = threading.Lock()

        self._fms_store = dict()
        self._fms_store['d'] = ModbusSequentialDataBlock(19000, [0] * 100)
        self._fms_store['c'] = ModbusSequentialDataBlock(9000, [0] * 100)
        self._fms_store['i'] = ModbusSequentialDataBlock(39000, [0] * 100)
        self._fms_store['h'] = ModbusSequentialDataBlock(49000, [0] * 100)

        self._sros_store = dict()
        self._sros_store['d'] = ModbusSequentialDataBlock(10000, [0] * 1000)
        self._sros_store['c'] = ModbusSequentialDataBlock(0, [0] * 1000)
        self._sros_store['i'] = ModbusSequentialDataBlock(30000, [0] * 1000)
        self._sros_store['h'] = ModbusSequentialDataBlock(40000, [0] * 1000)

        self.buffer_list = list()

        t = threading.Thread(target=self.loop_get_vehicle_state_from_fms, args=(self.__fms_api, self._fms_store,))
        t.setDaemon(True)
        t.start()

        t1 = threading.Thread(target=self.loop_get_vehicle_state_from_sros, args=(self._sros_store,))
        t1.setDaemon(True)
        t1.start()

    def loop_get_vehicle_state_from_fms(self, fms_api, _fms_store):
        while True:
            try:
                if not self._client.is_socket_open():
                    self._client.connect()

                if fms_api.init():
                    self.hfms = True
                    break
                time.sleep(10)
            except Exception as e:
                _logger.error(e)
                time.sleep(5)

        while True:
            try:
                if not self._client.is_socket_open():
                    self._client.connect()
                if not self.hfms:
                    return
                # 获取上线状态
                # _logger.info("-----------start loop_get_vehicle_state_from_fms-----------")

                vehicle_status = fms_api.get_vehicle_status()
                _fms_store['c'].setValues(9006, False if vehicle_status['is_online'] == 0 else True)

                vehicle_cur_excute_order_id = fms_api.get_vehicle_cur_excute_order_id()
                self.__cur_vehicle_cur_excute_order_id = vehicle_cur_excute_order_id
                _fms_store['i'].setValues(39001,
                                          [vehicle_cur_excute_order_id >> 16 & 0xFFFF,
                                           vehicle_cur_excute_order_id & 0xFFFF])
                if vehicle_cur_excute_order_id != 0:
                    vehicle_cur_excute_order_state = fms_api.get_vehicle_order_state(vehicle_cur_excute_order_id)
                    order_state = vehicle_cur_excute_order_state['order_state']
                    state = 0
                    if order_state == 'QUEUEING':
                        state = 1
                    elif order_state == 'CANCELLED':
                        state = 2
                    elif order_state == 'EXECUTING':
                        state = 3
                    elif order_state == 'FAILED':
                        state = 4
                    elif order_state == 'SUCCESS':
                        state = 5
                    elif order_state == 'DELETED':
                        state = 6
                    elif order_state == 'HELD':
                        state = 7
                    elif order_state == 'REJECTED':
                        state = 8
                    elif order_state == 'HANG':
                        state = 9
                    elif order_state == 'QUEUED':
                        state = 10
                    elif order_state == 'QUEUEING_HELD':
                        state = 11
                    _fms_store['i'].setValues(39003, state)
                else:
                    _fms_store['i'].setValues(39003, 0)
                self.__vehicle_queueing_order_list = fms_api.get_vehicle_queueing_order_list()
                for i in range(0, 5):
                    id = 0
                    if len(self.__vehicle_queueing_order_list) > i:
                        id = self.__vehicle_queueing_order_list[i]
                    _fms_store['i'].setValues(39017 + i * 2, [id >> 16 & 0xFFFF, id & 0xFFFF])
                time.sleep(1 / self.__get_fms_state_fps)
            except Exception as e:
                _logger.error(e)
                time.sleep(5)

    # 定时处理触摸屏请求
    def loop_get_vehicle_state_from_sros(self, _sros_store):
        while True:
            try:
                if not self._client.is_socket_open():
                    self._client.connect()
                # _logger.info('<<<<<<<<<<<<<<<<< start to get state from sros >>>>>>>>>>>>>>>>>')
                if len(self.buffer_list) > 10:
                    self._lock.acquire()
                    request_list = copy.deepcopy(self.buffer_list)
                    self.buffer_list.clear()
                    self._lock.release()
                    # _logger.info('request_list >>>>>>>>>>>>>>>>> %s' % request_list)
                    for item in request_list:
                        result = self.__get_callbacks[item[0]](item[1], item[2])
                        # _logger.info(
                        #     'loop_get_vehicle_state_from_sros:%s,%s' % (
                        #         str(item[1]), str(self.__extract_result(item[0], result))))
                        _sros_store[item[0]].setValues(
                            item[1], self.__extract_result(item[0], result))

                time.sleep(1 / self.__get_fms_state_fps)
            except Exception as e:
                _logger.error(e)
                time.sleep(5)

    def reset(self):
        ''' Resets all the datastores to their default values '''
        raise NotImplementedException()

    def validate(self, fx, address, count=1):
        ''' Validates the request to make sure it is in range

        :param fx: The function we are working with
        :param address: The starting address
        :param count: The number of values to test
        :returns: True if the request in within range, False otherwise
        '''
        # print("validate[%d] %d:%d" % (fx, address, count))

        if (address > 9000 and address < 10000) or (address > 19000 and address < 20000) or (
                address > 39000 and address < 40000) or (address > 49000 and address < 50000):
            return self._fms_store[self.decode(fx)].validate(address, count)

        if (10000 < address < 11000) or (0 < address < 1000) or (30000 < address < 31000) or (
                40000 < address < 41000):
            # _logger.info('validate _sros_store fx:%d, address:%d, count:%d' % (fx, address, count))
            return self._sros_store[self.decode(fx)].validate(address, count)

        return self._sros_store[self.decode(fx)].validate(address, count)

    def getValues(self, fx, address, count=1):
        ''' Get `count` values from datastore

        :param fx: The function we are working with
        :param address: The starting address
        :param count: The number of values to retrieve
        :returns: The requested values from a:a+c
        '''
        # TODO deal with deferreds
        # print("get values[%d] %d:%d" % (fx, address, count))
        # _logger.info('get values fx:%d, address:%d, count:%d' % (fx, address, count))
        if (address > 9000 and address < 10000) or (address > 19000 and address < 20000) or (
                address > 39000 and address < 40000) or (address > 49000 and address < 50000):
            # _logger.info('get result @@@@@@@ _fms_store fx:%d, address:%d, count:%d' % (fx, address, count))
            return self._fms_store[self.decode(fx)].getValues(address, count)

        if (10000 < address < 11000) or (0 < address < 1000) or (30000 < address < 31000) or (
                40000 < address < 41000):
            # _logger.info('get result @@@@@@@ _sros_store fx:%d, address:%d, count:%d' % (fx, address, count))
            self._lock.acquire()
            self.buffer_list.append([self.decode(fx), address, count])
            self._lock.release()
            return self._sros_store[self.decode(fx)].getValues(address, count)

        result = self.__get_callbacks[self.decode(fx)](address, count)
        # _logger.info('get result %s' % str(self.__extract_result(self.decode(fx), result)))
        return self.__extract_result(self.decode(fx), result)

    def setValues(self, fx, address, values):
        ''' Sets the datastore with the supplied values

        :param fx: The function we are working with
        :param address: The starting address
        :param values: The new values to be set
        '''

        # _logger.info('set values fx:%d, address:%d, values:%s' % (fx, address, str(values)))
        if (9000 < address < 10000) or (19000 < address < 20000) or (39000 < address < 40000) or (
                49000 < address < 50000):
            # _logger.info('set values >>>>>>>>_fms_store:%d, address:%d, values:%s' % (fx, address, str(values)))
            self._fms_store[self.decode(fx)].setValues(address, values)
            # todo
            if address == 9091 and values[0]:
                _logger.info('重启无线网卡>>>>>>')
                t = threading.Thread(target=Vc400Xpico().software_restart, )
                t.start()
            elif address == 9092 and values[0]:
                _logger.info('重置无线网卡>>>>>>')
                t = threading.Thread(target=Vc400Xpico().reset_Xpico, )
                t.start()
            elif address == 9090:
                _logger.info('开启关闭无线网卡>>>>>>')
                param1 = 1 if values[0] else 0
                t = threading.Thread(target=Vc400Xpico().down_or_up_xpico, args=(param1,))
                t.start()

            if not self.hfms:
                return
            vehicle_status = self.__fms_api.get_vehicle_status()

            if address == 49001 and len(values) == 2 and self.__cur_vehicle_cur_excute_order_id == 0 and vehicle_status[
                'is_online'] == 1 and len(self.__fms_api.get_vehicle_queueing_order_list()) == 0:
                template_id = (values[0] << 16) + (values[1] & 0xFFFF)
                t = threading.Thread(target=self.__fms_api.add_template_order_for_vehicle, args=(template_id,))
                t.start()
            elif address == 49003 and len(values) == 2:  # 取消订单
                order_id = (values[0] << 16) + (values[1] & 0xFFFF)
                for id in self.__vehicle_queueing_order_list:
                    if (id & 0xFFFFFFFF) == order_id:
                        t = threading.Thread(target=self.__fms_api.operate_vehicle_cur_order,
                                             args=(id, 'CMD_ORDER_CANCEL',))
                        t.start()
                        break
            elif address == 9001 and values[0]:
                t = threading.Thread(target=self.__fms_api.operate_vehicle_cur_order,
                                     args=(self.__cur_vehicle_cur_excute_order_id, 'CMD_ORDER_HELD',))
                t.start()
            elif address == 9002 and values[0]:
                t = threading.Thread(target=self.__fms_api.operate_vehicle_cur_order,
                                     args=(self.__cur_vehicle_cur_excute_order_id, 'CMD_ORDER_CONTINUE_FROM_HELD',))
                t.start()
            elif address == 9003 and values[0]:
                t = threading.Thread(target=self.__fms_api.operate_vehicle_cur_order,
                                     args=(self.__cur_vehicle_cur_excute_order_id, 'CMD_ORDER_CANCEL',))
                t.start()
            elif address == 9004 and values[0]:
                t = threading.Thread(target=self.__fms_api.operate_vehicle_cur_order,
                                     args=(self.__cur_vehicle_cur_excute_order_id, 'CMD_ORDER_JUMP_FROM_HANG',))
                t.start()
            elif address == 9005 and values[0]:
                t = threading.Thread(target=self.__fms_api.operate_vehicle_cur_order,
                                     args=(self.__cur_vehicle_cur_excute_order_id, 'CMD_ORDER_CONTINUE_FROM_HANG',))
                t.start()
            elif address == 9006:
                param1 = 1 if values[0] else 0
                t = threading.Thread(target=self.__fms_api.operate_vehicle,
                                     args=('CMD_VEHICLE_UPDATE_ONLINE', param1))
                t.start()
            elif address == 9007 and values[0]:
                t = threading.Thread(target=self.__fms_api.operate_vehicle_free_all, args=('COMMAND_FREE_ALL',))
                t.start()
            elif address == 9008 and values[0]:
                t = threading.Thread(target=self.__fms_api.operate_vehicle, args=('CMD_VEHICLE_CHARGE',))
                t.start()
            elif address == 9009 and values[0]:
                t = threading.Thread(target=self.__fms_api.operate_vehicle, args=('CMD_VEHICLE_GO_PARKING',))
                t.start()
            return

        self.__set_callbacks[self.decode(fx)](address, values)

    def __str__(self):
        ''' Returns a string representation of the context

        :returns: A string representation of the context
        '''
        return "Screen Slave Context"

    def __build_mapping(self):
        '''
        A quick helper method to build the function
        code mapper.
        '''
        kwargs = {}
        if self.unit:
            kwargs["unit"] = self.unit
        self.__get_callbacks = {
            'd': lambda a, c: self._client.read_discrete_inputs(a, c, **kwargs),
            'c': lambda a, c: self._client.read_coils(a, c, **kwargs),
            'h': lambda a, c: self._client.read_holding_registers(a, c, **kwargs),
            'i': lambda a, c: self._client.read_input_registers(a, c, **kwargs),
        }
        self.__set_callbacks = {
            'd': lambda a, v: self._client.write_coils(a, v, **kwargs),
            'c': lambda a, v: self._client.write_coils(a, v, **kwargs),
            'h': lambda a, v: self._client.write_registers(a, v, **kwargs),
            'i': lambda a, v: self._client.write_registers(a, v, **kwargs),
        }

    def __extract_result(self, fx, result):
        ''' A helper method to extract the values out of
        a response.  TODO make this consistent (values?)
        '''
        if not result.isError():
            if fx in ['d', 'c']:
                return result.bits
            if fx in ['h', 'i']:
                return result.registers
        else:
            return result


class FmsApi():
    def __init__(self):
        self._host = 'http://192.168.83.202:8088'
        self._vehicle_serial_no = '31746e6d92600c04'
        self._vehicle_id = 8
        self.order_id_list = None

    def init(self):
        try:
            # _logger.info('init')
            self._vehicle_serial_no = self.get_vehicle_serial_no()
            self._host = self.get_host()
            self._vehicle_id = self.get_vehicle_id(self._vehicle_serial_no)
            return True
        except Exception as e:
            return False

    def get_vehicle_serial_no(self):
        conn = sqlite3.connect('/sros/db/main.db3')
        cursor = conn.cursor()
        cursor.execute('SELECT value FROM config WHERE key="main.serial_no"')
        values = cursor.fetchall()
        vehicle_serial_no = values[0][0]
        # _logger.info('Get vehicle serial no ' + vehicle_serial_no)
        if vehicle_serial_no == 'NA':
            raise Exception('vehicle serial no is NA!')
        return vehicle_serial_no

    def get_host(self):
        conn = sqlite3.connect('/sros/db/main.db3')
        cursor = conn.cursor()
        cursor.execute('SELECT value FROM config WHERE key="network.server_ip"')
        values = cursor.fetchall()
        ip = values[0][0]
        # _logger.info('Get server ip ' + ip)
        if ip == 'NA':
            raise Exception('Server ip is NA!')
        cursor.execute('SELECT value FROM config WHERE key="network.fms_http_server_port"')
        values = cursor.fetchall()
        port = values[0][0]
        # _logger.info('Get server port ' + port)
        host = 'http://' + ip + ':' + port
        # _logger.info('host is ' + host)
        return host

    def get_vehicle_id(self, serial_no):
        req = urllib.request.Request(self._host + '/api/v2/vehicles/serial/' + serial_no)
        req.add_header('token', 'YWRtaW4sMTg5MDcyMTUyNTQ1NCw3MTAxZDI5N2M0OWIxNTdjMzU2MzM2ZTNmYWViOTE3Ng==')
        r = urllib.request.urlopen(req)
        d = json.loads(r.read().decode('utf-8'))
        # _logger.info('get vehicle id ' + str(d['id']))
        return d['id']

    def get_vehicle_status(self):
        req = urllib.request.Request(self._host + '/api/v2/vehicles/' + str(self._vehicle_id))
        req.add_header('token', 'YWRtaW4sMTg5MDcyMTUyNTQ1NCw3MTAxZDI5N2M0OWIxNTdjMzU2MzM2ZTNmYWViOTE3Ng==')
        r = urllib.request.urlopen(req)
        d = json.loads(r.read().decode('utf-8'))
        return d

    def get_vehicle_cur_excute_order_id(self):
        # print('get_vehicle_cur_excute_order_id')
        req = urllib.request.Request(self._host + '/api/v2/vehicles/serial/' + self._vehicle_serial_no)
        req.add_header('token', 'YWRtaW4sMTg5MDcyMTUyNTQ1NCw3MTAxZDI5N2M0OWIxNTdjMzU2MzM2ZTNmYWViOTE3Ng==')
        r = urllib.request.urlopen(req)
        d = json.loads(r.read().decode('utf-8'))
        if int(d['execute_order_id']) != 0:
            _logger.info('get vehicle cur excute order id ' + d['execute_order_id'])
        return int(d['execute_order_id'])

    def get_vehicle_order_state(self, order_id):
        req = urllib.request.Request(self._host + '/api/v2/orders/' + str(order_id))
        req.add_header('token', 'YWRtaW4sMTg5MDcyMTUyNTQ1NCw3MTAxZDI5N2M0OWIxNTdjMzU2MzM2ZTNmYWViOTE3Ng==')
        r = urllib.request.urlopen(req)
        d = json.loads(r.read().decode('utf-8'))
        # _logger.info('get order state is ' + d['order_state'])

        return d

    def get_vehicle_queueing_order_list(self):
        # print('get_vehicle_queueing_order_list()')
        params = urllib.parse.urlencode(
            {'perpage': 5, 'filter_by_state': OrderState.QUEUEING.value, 'appoint_vehicle_id': self._vehicle_id})
        req = urllib.request.Request(self._host + '/api/v2/orders?' + params)
        req.add_header('token', 'YWRtaW4sMTg5MDcyMTUyNTQ1NCw3MTAxZDI5N2M0OWIxNTdjMzU2MzM2ZTNmYWViOTE3Ng==')
        r = urllib.request.urlopen(req)
        d = json.loads(r.read().decode('utf-8'))
        orders = d['orders']
        order_id_list = []
        for order in orders:
            order_id_list.append(int(order['id']))
        if order_id_list and order_id_list != self.order_id_list:
            # _logger.info('get vehicle id queue id ' + str(order_id_list))
            self.order_id_list = order_id_list
        return order_id_list

    def add_template_order_for_vehicle(self, order_template_id):
        # _logger.info('start add template order for vehicle, order template id is ' + str(order_template_id))
        req = urllib.request.Request(self._host + '/api/v2/orders/template/' + str(order_template_id))
        req.add_header('token', 'YWRtaW4sMTg5MDcyMTUyNTQ1NCw3MTAxZDI5N2M0OWIxNTdjMzU2MzM2ZTNmYWViOTE3Ng==')
        req.add_header('Content-Type', 'application/json')
        # data = urllib.parse.urlencode({'vehicle_id': self._vehicle_id})
        # data = data.encode('ascii')
        # print(data)
        data = {'vehicle_id': self._vehicle_id}
        r = urllib.request.urlopen(req, json.dumps(data).encode("utf-8"))
        d = json.loads(r.read().decode('utf-8'))

    def operate_vehicle_cur_order(self, order_id, command, param1=None):
        '''
        操作当前车车辆订单
        :param order_id:
        :param command:  CMD_ORDER_HELD               订单暂停
                         CMD_ORDER_CONTINUE_FROM_HELD 订单继续
                         CMD_ORDER_CANCEL             订单取消
                         CMD_ORDER_JUMP_FROM_HANG     跳过任务
                         CMD_ORDER_CONTINUE_FROM_HANG 重试任务
        :param param1: 参数1
        :return:
        '''
        # _logger.info('request command %s' % command)
        req = urllib.request.Request(self._host + '/api/v2/orders/' + str(order_id) + '/command')
        req.add_header('token', 'YWRtaW4sMTg5MDcyMTUyNTQ1NCw3MTAxZDI5N2M0OWIxNTdjMzU2MzM2ZTNmYWViOTE3Ng==')
        req.add_header('Content-Type', 'application/json')
        param = {'command_type': command}
        if param1 is not None:
            param['param1'] = param1
        data = json.dumps(param)
        response = urllib.request.urlopen(req, data.encode('utf-8'))
        # _logger.info('response status code %s' % response.status)

    def operate_vehicle(self, command, param1=None):
        '''
        操作当前车车辆订单
        :param order_id:
        :param command:  CMD_VEHICLE_UPDATE_ONLINE 上线下线
                         CMD_VEHICLE_CHARGE        返回充电
        :param param1: 参数1
        :return:
        '''
        # _logger.info('request command %s' % command)
        req = urllib.request.Request(self._host + '/api/v2/vehicles/command')
        req.add_header('token', 'YWRtaW4sMTg5MDcyMTUyNTQ1NCw3MTAxZDI5N2M0OWIxNTdjMzU2MzM2ZTNmYWViOTE3Ng==')
        req.add_header('Content-Type', 'application/json')
        param = {'vehicle_id': self._vehicle_id, 'command_type': command}
        if param1 is not None:
            param['param1'] = param1
        data = json.dumps(param)
        response = urllib.request.urlopen(req, data.encode('utf-8'))
        # _logger.info('response status code %s' % response.status)

    def operate_vehicle_free_all(self, command, param1=None):
        '''
        操作当前车辆清楚路径占用
        :param order_id:
        :param command:  COMMAND_FREE_ALL 清除路径占用
        :param param1: 参数1
        :return:
        '''
        # _logger.info('request command %s' % command)
        req = urllib.request.Request(self._host + '/api/v2/vehicles/resource')
        req.add_header('token', 'YWRtaW4sMTg5MDcyMTUyNTQ1NCw3MTAxZDI5N2M0OWIxNTdjMzU2MzM2ZTNmYWViOTE3Ng==')
        req.add_header('Content-Type', 'application/json')
        param = {'vehicle_id': self._vehicle_id, 'command_type': command}
        if param1 is not None:
            param['param1'] = param1
        data = json.dumps(param)
        response = urllib.request.urlopen(req, data.encode('utf-8'))
        # _logger.info('response status code %s' % response.status)


class ScreenV2():
    def __init__(self):
        conn = sqlite3.connect('/sros/db/main.db3')
        cursor = conn.cursor()
        cursor.execute('SELECT value FROM config WHERE key="modbus.modbus_tcp_port"')
        values = cursor.fetchall()
        port = values[0][0]
        # _logger.info('Get port ' + port)

        cursor.execute('SELECT value FROM config WHERE key="hmi.touch_screen_get_fms_state_fps"')
        values = cursor.fetchall()
        get_fms_state_fps = 1
        try:
            get_fms_state_fps = int(values[0][0]) / 1000
        except Exception as e:
            _logger.error(e)
        if get_fms_state_fps < 0.2:
            get_fms_state_fps = 0.2
        elif get_fms_state_fps > 2:
            get_fms_state_fps = 2

        self.__fms_api = FmsApi()
        # 由于是内部tcp超时时间不需要很长，长了话由于pymodbus库有问题，假设在阻塞的时候，串口一次收到了两帧数据，就会导致只处理第一帧，第二帧会收到第三帧的时候才会处理
        self.__client = ModbusTcpClient(port=port, timeout=1)
        self.__context = ScreenSlaveContext(self.__client, self.__fms_api, get_fms_state_fps)

    def run_server(self):
        conn = sqlite3.connect('/sros/db/main.db3')
        cursor = conn.cursor()
        cursor.execute('SELECT value FROM config WHERE key="hmi.touch_screen_device_name"')
        values = cursor.fetchall()
        screen_device_name = values[0][0]
        # _logger.info('Get screen_device_name ' + screen_device_name)

        cursor.execute('SELECT value FROM config WHERE key="hmi.touch_screen_device_rate"')
        values = cursor.fetchall()
        baudrate = int(values[0][0])
        # _logger.info('Get server baudrate ' + str(baudrate))

        # _logger.info('run server')
        context = ModbusServerContext(slaves=self.__context, single=True)
        # _logger.info('start server')
        # StartTcpServer(context, address=("0.0.0.0", 5022))
        StartSerialServer(context, framer=ModbusRtuFramer, port=screen_device_name, timeout=.05, baudrate=baudrate)


class Vc400Xpico():
    # 重启无线网卡   9091  发 1 重启
    # 重置无线网卡  9092    发 1 重置
    # 开启关闭无线网卡 9090   开启 1 关闭 0

    def __init__(self):
        self.ip = "169.254.0.1"
        self.port = 8080
        self.auth = HTTPDigestAuth("admin", "PASSWORD")

        # 导出配置文件url
        self.export_conf_url = "http://{0}:{1}/export/config".format(self.ip, self.port)
        # 导入配置文件url
        self.import_conf_url = "http://{0}:{1}/import/config".format(self.ip, self.port)
        # 导出状态文件url
        self.export_status_url = "http://{0}:{1}/export/status".format(self.ip, self.port)
        # 采取状态行动
        self.action_status_url = "http://{0}:{1}/action/status".format(self.ip, self.port)

        self.ap_state_path = "/sros/web/ui-server/vc400/xml_package/children/setting_ap.xml"
        self.bridge_xml_path = "/sros/web/ui-server/vc400/xml_package/children/change_bridge.xml"

    def reset_Xpico(self):
        _logger.info("reset_Xpico is start...")
        os.system("sh /sros/web/ui-server/vc400/sh_package/start_ap_conf.sh")

    def down_or_up_xpico(self, state):
        _logger.info("state : %s" % str(state))
        if int(state) == 1:
            self.modify_setting_ap("Enabled")
        elif int(state) == 0:
            self.modify_setting_ap("Disabled")

    def software_restart(self):
        """重启设备"""
        _logger.info("software_restart is start...")
        data = {"group": "Device", "action": "Reboot", }
        res = requests.post(self.action_status_url, data=data, auth=self.auth, timeout=30)

        # self.reboot_asix()
        # if os.system("sh /sros/web/ui-server/vc400/sh_package/software_restart.sh") != 0:
        #     return {"code": 400, "msg": "restart is failed!"}
        return {"code": res.status_code}

    def modify_setting_ap(self, state):
        res = self.setting_ap(self.ap_state_path, state)
        if not res:
            _logger.info("modify_setting_ap is error!")
            return False
        if not self.import_config_xml(self.ap_state_path):
            _logger.info("down_or_up_ap import conf is error!")
            return False

        if not self.modify_bridge("ap0", self.bridge_xml_path, state):
            _logger.error("push_ap_conf modify_bridge is error!")
            return False
        if not self.import_config_xml(self.bridge_xml_path):
            _logger.error("push_ap_conf import bridge conf is error!")
            return False

        os.system("sh /sros/web/ui-server/vc400/sh_package/ap_conf.sh")
        return True

    def modify_bridge(self, bridge_obj, bridge_conf_path, state):
        """
        bridge_obj :桥接到 ap0 还是 wlan0
        """
        try:
            tree = etree.parse(bridge_conf_path)
            for item in tree.xpath("//configgroup"):
                if item.attrib["name"] == "Bridge":
                    if bridge_obj == "ap0":
                        for child in item:
                            if child.attrib["name"] == "Interface":
                                child[0].text = bridge_obj  # interface

                            elif child.attrib["name"] == "Ethernet":
                                for sun in child:
                                    if sun.attrib["name"] == "State":
                                        sun.text = state

            self.write_config_xml(tree, bridge_conf_path)
        except Exception as e:
            _logger.error(e)
            return False
        return True

    def setting_ap(self, ap_state_path, state):
        """开启关闭ap state: Enabled/Disabled"""
        try:
            tree = etree.parse(ap_state_path)
            for item in tree.xpath("//configgroup"):
                if item.attrib["name"] == "Interface" and item.attrib["instance"] == "ap0":
                    for child in item:
                        if child.attrib["name"] == "State":
                            child[0].text = state
            self.write_config_xml(tree, ap_state_path)
        except Exception as e:
            _logger.error(e)
            return False
        return True

    def import_config_xml(self, conf_path):
        """导入配置文件"""
        with open(conf_path, "r") as f:
            f_read = f.read()
        files = {"configrecord": f_read}
        res = requests.post(url=self.import_conf_url, files=files, auth=self.auth, timeout=30)
        config_xml = str(res.content, encoding="utf-8")
        # config_xml = str(res.content)
        root = etree.XML(config_xml)
        if root.xpath("//result")[0].text != "Succeeded":
            _logger.error(root.xpath("//result")[0].text)
            return False
        return True

    def write_config_xml(self, tree, out_path):
        '''''将xml文件写出
          tree: xml树
          out_path: 写出路径'''
        DOCTYPE = '''<!DOCTYPE configrecord [<!ELEMENT configrecord (configgroup+)>
                                            <!ELEMENT configgroup (configitem+)>
                                            <!ELEMENT configitem (value+)>
                                            <!ELEMENT value (#PCDATA)>
                                            <!ATTLIST configrecord version CDATA #IMPLIED>
                                            <!ATTLIST configgroup name CDATA #IMPLIED>
                                            <!ATTLIST configgroup instance CDATA #IMPLIED>
                                            <!ATTLIST configitem name CDATA #IMPLIED>
                                            <!ATTLIST configitem instance CDATA #IMPLIED>
                                            <!ATTLIST value name CDATA #IMPLIED>
                                            ]>'''

        tree.write(out_path, standalone="yes", method="xml", doctype=DOCTYPE, xml_declaration=True)

        with open(out_path, "r") as f:
            xml_bool = f.read()
        with open(out_path, "w") as t:
            t.write(xml_bool.replace("encoding='ASCII'", ""))


if __name__ == '__main__':
    sros_log = sros_log.SrosLog("screen_v2")
    sros_log.sendLogToFile(logging.INFO)
    # sros_log.sendLogToConsole()

    screen = ScreenV2()
    screen.run_server()

    # fms_api = FmsApi()
    # fms_api.init()
    # fms_api.add_template_order_for_vehicle(1)
    # fms_api.operate_vehicle_cur_order(117, 'CMD_ORDER_CANCEL')
