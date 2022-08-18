#!/usr/bin/env python
# -*- coding:utf-8 -*-
# file modbus_rtu_test.py
# author pengjiali
# date 19-10-29.
# copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# describe

from pymodbus.client.sync import ModbusSerialClient
import unittest
import random
import time
import logging

FORMAT = ('%(asctime)-15s %(threadName)-15s'
          ' %(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
logging.basicConfig(format=FORMAT)
log = logging.getLogger()
log.setLevel(logging.DEBUG)


class TestModbusRTU(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        unittest.TestCase.__init__(self, *args, **kwargs)

    @classmethod
    def setUpClass(self):
        self._client = ModbusSerialClient(method='rtu', port='/dev/ttyUSB0', timeout=.2, baudrate=9600)
        ret = self._client.connect()
        print('connect to serial port ', ret, '\n')

    @classmethod
    def tearDownClass(self):
        pass

    def setUp(self):
        # 每个测试用例执行之前做操作
        pass

    def tearDown(self):
        # 每个测试用例执行之后做操作
        pass

    def test_get_ip(self):
        for i in range(1):
            ret = self._client.read_input_registers(30049, 4)
            self.assertNotIsInstance(ret, Exception)
            print('get ip', ret.registers)
            self.assertNotEqual(ret.registers, [0, 0, 0, 0])

    def test_get_system_state(self):
        ret = self._client.read_input_registers(30001, 1)
        self.assertNotIsInstance(ret, Exception)
        print('get state', ret.registers)

    def test_start_location_by_station(self):
        ret = self._client.write_registers(40028, 12)
        self.assertNotIsInstance(ret, Exception)


if __name__ == '__main__':
    unittest.main()  # 运行所有的测试用例
