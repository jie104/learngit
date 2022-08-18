#!/usr/bin/env python
# -*- coding:utf-8 -*-
# file huawei_odu.py
# author pengjiali
# date 19-10-20.
# copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# describe odu温循线项目

'''
NOTE 所有的上料和下料，都是给机台上料下料

保持寄存器：
40033： 驱动心跳
40034： 下料台状态
    0 Disconnect ：链接不上
    1 Ready ： 可以上料
    2 Loading ： 储料中
    3 Loadeded ： 料满，可以下料
    4 Busy ： 在上下料过程中
    0x64 Error ： 故障
    0x0C Error_SafeDoorOpen : 安全门打开
40035： 上料状态
    0 NoneCommand ： 没有命令， 由matrix写入
    1 StartSend ： 开始启动
    2 Starting ： 启动中
    3 StartSucceed ： 启动成功
    4 StartFailed ： 启动失败
40036： 上料命令
    1 上料
40037： 下料状态
    0 NoneCommand ： 没有命令， 由matrix写入
    1 StartSend ： 开始启动
    2 Starting ： 启动中
    3 StartSucceed ： 启动成功
    4 StartFailed ： 启动失败
40038： 下料命令
    1 下料

'''

from socket import *
from enum import Enum
from pymodbus.client.sync import ModbusTcpClient
import time
import ctypes
import logging

FORMAT = ('%(asctime)-15s %(threadName)-15s'
          ' %(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
logging.basicConfig(format=FORMAT, filename='huawei.log')
log = logging.getLogger()
log.setLevel(logging.INFO)


class FeedingTableState(Enum):
    Disconnect = 0  # 链接不上
    Ready = 1  # 可以上货
    Loading = 2  # 储料中
    Loadeded = 3  # 料满，可以下料
    Busy = 4  # 在上下料过程中
    Error = 0x64
    Error_SafeDoorOpen = 0x0C


class CommandStartState(Enum):
    '''
    命令启动状态
    '''
    NoneCommand = 0  # 没有命令， 由matrix写入
    StartSend = 1  # 开始启动
    Starting = 2  # 启动中
    StartSucceed = 3  # 启动成功
    StartFailed = 4  # 启动失败


class ModbusDefine(Enum):
    '''
    寄存器地址定义
    '''
    DriverCount = 0  # 本驱动的心跳 (40033)
    FeedingTableState = 1  # 下料台的状态（40034）

    LoadStartState = 2  # 上料状态（40035）
    Load = 3  # 上料命令（40036）

    UnloadStartState = 4  # 下料状态 （4003)7）
    Unload = 5  # 下料命令 （40038）


class FeedingTable():
    def __init__(self):
        self._client = socket(AF_INET, SOCK_DGRAM)
        self._client.settimeout(3)
        self._host = '192.168.100.168'
        self._port = 1025
        self._bufsize = 18
        self._addr = (self._host, self._port)

    def __response_check(self, response):
        if len(response) != 11:
            # raise Exception("Response no check! len is", len(response), response)
            pass
        if response[0] != 0x68:
            raise Exception('Head not check!', response)
        # if response[10] != 0xFE:
        #     raise Exception('botton not check', response)

    def get_state(self):
        self._client.sendto(b"\x68\x06\x00\x00\x03\x06\x00\x77\xFE", self._addr)
        response, ADDR = self._client.recvfrom(self._bufsize)
        # log.info(response, len(response))
        self.__response_check(response)
        if response[6] == 0x01:
            raise Exception('Get state error! error code is 0x%X' % ((response[7] << 16) + response[8]))
        return response[8]

    def load(self):
        """
        给机台上料
        :return: 上料成功返回True，上料失败返回False，上料异常抛出异常
        """
        log.info('上货架')
        self._client.sendto(b"\x68\x06\x00\x00\x03\x07\x00\x78\xFE", self._addr)
        response, ADDR = self._client.recvfrom(self._bufsize)
        self.__response_check(response)
        if response[6] == 0x01:
            log.info('Load failed! error code is 0x%X' % ((response[7] << 16) + response[8]))
            return False
        return True

    def unload(self):
        '''下货架'''
        log.info('下货架')
        self._client.sendto(b"\x68\x06\x00\x00\x03\x08\x00\x79\xFE", self._addr)
        response, ADDR = self._client.recvfrom(self._bufsize)
        self.__response_check(response)
        if response[6] == 0x01:
            log.info('Load failed! error code is 0x%X' % ((response[7] << 16) + response[8]))
            return False
        return True


class Translator():
    '''
    将华为顶一顶私有协议转换为modbus协议
    '''

    def __init__(self):
        self._feeding_table = FeedingTable()
        self._modbus = ModbusTcpClient()
        # self._modbus = ModbusTcpClient(host='192.168.100.5')

        self._user_define_register_addr = 40033  # 用户定义寄存器的起始地址

        self._feeding_table_state = FeedingTableState.Disconnect
        self._count = ctypes.c_uint16()  # 计数器，链接上了modbus就写一次
        self._load_command_start_state = CommandStartState.NoneCommand
        self._unload_command_start_state = CommandStartState.NoneCommand

    def run(self):
        log.info('start run')
        try:
            # NOTE: 放到VC300中跑时，本脚本启动的同时，用户寄存器是初始值，所以没有必要手动设置，反倒是sros可能没启动，会导致写寄存器失败
            self._modbus.write_registers(self._user_define_register_addr, [0, 0, 0, 0, 0, 0])
        except Exception as e:
            log.info(e)

        while True:
            if not self._modbus.is_socket_open():
                try:
                    ret = self._modbus.connect()
                    if not ret:
                        log.info('modbus connnected failed!')
                        time.sleep(1)
                        continue
                except Exception as e:
                    log.info(e)
                    time.sleep(1)
                    continue

            ret = self._modbus.read_holding_registers(self._user_define_register_addr, 6)
            registers = ret.registers
            # log.info(registers)

            try:
                self._feeding_table_state = self._feeding_table.get_state()
            except Exception as e:  # 获取状态失败
                self._feeding_table_state = FeedingTableState.Disconnect.value
                log.info(e)

            if registers[ModbusDefine.Load.value] != 0:
                # 收到mission需要上料的请求
                log.info('load')
                self._load_command_start_state = CommandStartState.StartSend
                self._modbus.write_registers(self._user_define_register_addr + ModbusDefine.LoadStartState.value,
                                             [self._load_command_start_state.value, 0])
                try:
                    ret = self._feeding_table.load()
                    if ret:
                        self._load_command_start_state = CommandStartState.StartSucceed
                    else:
                        self._load_command_start_state = CommandStartState.StartFailed
                    self._modbus.write_register(self._user_define_register_addr + ModbusDefine.LoadStartState.value,
                                                self._load_command_start_state.value)
                except Exception as e:
                    # 上料失败
                    log.info(e)
                    self._load_command_start_state = CommandStartState.Starting
                    self._modbus.write_register(self._user_define_register_addr + ModbusDefine.LoadStartState.value,
                                                self._load_command_start_state.value)

            if registers[ModbusDefine.Unload.value] != 0:
                # 收到mission需要上料的请求
                log.info('unload')
                self._unload_command_start_state = CommandStartState.StartSend
                self._modbus.write_registers(self._user_define_register_addr + ModbusDefine.UnloadStartState.value,
                                             [self._unload_command_start_state.value, 0])
                try:
                    ret = self._feeding_table.unload()
                    if ret:
                        self._unload_command_start_state = CommandStartState.StartSucceed
                    else:
                        self._unload_command_start_state = CommandStartState.StartFailed
                    self._modbus.write_register(self._user_define_register_addr + ModbusDefine.UnloadStartState.value,
                                                self._unload_command_start_state.value)
                except Exception as e:
                    # 上料失败
                    log.info(e)
                    self._unload_command_start_state = CommandStartState.Starting
                    self._modbus.write_register(self._user_define_register_addr + ModbusDefine.UnloadStartState.value,
                                                self._unload_command_start_state.value)

            self._count.value += 1
            self._modbus.write_registers(self._user_define_register_addr,
                                         [self._count.value, self._feeding_table_state])

            time.sleep(0.1)

    def frist_line_unload_test(self):
        self._modbus.write_registers(40017, [20, 3, 0])
        self._feeding_table.unload();

    def frist_line_load_test(self):
        self._feeding_table.load();
        self._modbus.write_registers(40017, [20, 4, 0])

    def modbus_write_test(self):
        # self._modbus.write_registers(self._user_define_register_addr,
        #                              [1, 2, 3, 4, 5, 6])
        ret = self._modbus.read_holding_registers(self._user_define_register_addr, 6)
        registers = ret.registers
        log.info(registers)


if __name__ == '__main__':
    log.info('start huawei_odu.py')
    translator = Translator()
    # translator.modbus_write_test()
    # translator.frist_line_load_test()
    # translator.frist_line_unload_test()
    translator.run()
