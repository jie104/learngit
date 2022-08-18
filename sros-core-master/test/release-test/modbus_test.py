#!/usr/bin/env python
# -*- coding:utf-8 -*-
# file modbus_test.py
# author pengjiali
# date 19-9-26.
# copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# describe

from pymodbus.client.sync import ModbusTcpClient
import unittest
import random
import time

class TestModbus(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        unittest.TestCase.__init__(self, *args, **kwargs)
        self._tcp_client = ModbusTcpClient('192.168.30.114')

    @classmethod
    def setUpClass(self):
        pass

    @classmethod
    def tearDownClass(self):
        pass

    def setUp(self):
        # 每个测试用例执行之前做操作
        pass

    def tearDown(self):
        # 每个测试用例执行之后做操作
        pass

    def test_set_gpio_bit(self):
        for i in range(10):
            value = random.choice([True, False]);
            index = random.randint(0, 7)
            print('set gpio output bit', index, '->', value)
            self._tcp_client.write_coil(33 + index, value)
            for t in range(5):
                time.sleep(0.1)
                result = self._tcp_client.read_discrete_inputs(10033, 8)
                if result.bits[index] == value:
                    break
            self.assertEqual(result.bits[index], value)

    def test_set_gpio_value(self):
        for i in range(10):
            value = random.randint(0x00, 0xFF)
            print('set gpio output value', '->', '%#x'%value)
            self._tcp_client.write_registers(40030, [value, 0xFFFF])
            for t in range(5):
                time.sleep(0.1)
                result = self._tcp_client.read_input_registers(30022)
                if result.registers[0] == value:
                    break
            self.assertEqual(result.registers[0], value)

    def test_get_ip(self):
        ret = self._tcp_client.read_input_registers(30049, 4)
        print('get ip', ret.registers)
        self.assertNotEqual(ret.registers, [0, 0, 0, 0])



if __name__ == '__main__':
    unittest.main()  # 运行所有的测试用例