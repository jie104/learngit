#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @File: eac_v2_test.py
# @Author: pengjiali
# @Date: 19-11-19
# @Copyright: Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# @Describe:

import unittest
import sys
import os

sys.path.append('../communication')
import asyncio
import time
from srp_unit import srp_unit, srp
import subprocess
from pymodbus.client.sync import ModbusTcpClient

pwd = os.path.split(os.path.realpath(__file__))[0]
print(pwd)
eac = subprocess.Popen(['python3', pwd + '/../simulate/eac_v2_modbus_tcp_simulate.py'])


class EacV2Test(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        srp_unit.backup_config('device.enable_eac')
        srp_unit.backup_config('device.eac_communication_type')
        srp_unit.backup_config('device.eac_ip')
        srp_unit.backup_config('device.eac_port')
        srp_unit.backup_config('device.eac_basis_input_register_addr')
        srp_unit.backup_config('device.eac_basis_hold_register_addr')
        srp_unit.backup_config('device.eac_slave_id')

        srp.set_config({'device.enable_eac': "True"})
        srp.set_config({'device.eac_communication_type': "modbus_tcp"})
        srp.set_config({'device.eac_ip': '192.168.83.189'})
        srp.set_config({'device.eac_port': '5020'})
        srp.set_config({'device.eac_basis_input_register_addr': 0})
        srp.set_config({'device.eac_basis_hold_register_addr': 0})
        srp.set_config({'device.eac_slave_id': 1})

        print('waiting eac ok, 3s');
        time.sleep(3)

    @classmethod
    def tearDownClass(self):
        srp_unit.restore_config('device.enable_eac')
        srp_unit.restore_config('device.eac_communication_type')
        srp_unit.restore_config('device.eac_ip')
        srp_unit.restore_config('device.eac_port')
        srp_unit.restore_config('device.eac_basis_input_register_addr')
        srp_unit.restore_config('device.eac_basis_hold_register_addr')
        srp_unit.restore_config('device.eac_slave_id')

        eac.kill()

    def test_eac_system_state_is_ok(self):
        tcp_client = ModbusTcpClient('localhost', 5020)
        ret = tcp_client.read_input_registers(0, 1)
        print('eac system state', ret.registers)
        self.assertNotEqual(ret.registers, [0, ])
        tcp_client.close()

    def test_get_config(self):
        config = srp.get_config('hmi.speaker_volume')
        volume = int(config['value'])
        self.assertLessEqual(volume, 100)
        self.assertGreaterEqual(volume, 0)

    def test_get_config2(self):
        config = srp.get_config('hmi.speaker_volume')
        volume = int(config['value'])
        self.assertLessEqual(volume, 100)
        self.assertGreaterEqual(volume, 0)

    def test_eac_finish_immediately(self):
        def notify_action_task_finished_callback(action_task):
            srp_unit._async_result.accept(action_task)

        srp.set_notify_action_task_finished_callback(
            notify_action_task_finished_callback)

        async def func():
            for i in range(1, 2):
                srp_unit._async_result.clear()
                srp.excute_action_task(i, 195, 1, 0)
                print(i, 192, 1, 0)
                await srp_unit._async_result.wait()

        asyncio.run_coroutine_threadsafe(func(), srp_unit.loop).result(200)

    @unittest.skip('test disabled')
    def test_continuous_action(self):
        '''
        连续发送动作（发送动作，等待动作的Notification）
        :return: None
        '''

        def notify_action_task_finished_callback(action_task):
            srp_unit._async_result.accept(action_task)

        srp.set_notify_action_task_finished_callback(
            notify_action_task_finished_callback)

        async def func():
            for i in range(1, 10):
                srp_unit._async_result.clear()
                srp.excute_action_task(i, 192, 1, 0)
                print(i, 192, 1, 0)
                await srp_unit._async_result.wait()

        asyncio.run_coroutine_threadsafe(func(), srp_unit.loop).result(200)

    @unittest.skip('test disabled')
    def test_action_cancel(self):
        print('test_action_cancel')
        srp.excute_action_task(77, 192, 1, 1)
        time.sleep(3)
        srp.cancel_action_task()

    @unittest.skip('test disabled')
    def test_violence_continuous_start_action(self):
        for i in range(1, 1000):
            print("New action ", i, "(192, 1, ,1)")
            srp.async_excute_action_task(i, 192, 1, 1)
            pass

        # 等待任务结束
        def notify_action_task_finished_callback(action_task):
            srp_unit._async_result.accept(action_task)

        srp.set_notify_action_task_finished_callback(
            notify_action_task_finished_callback)

        async def func():
            await srp_unit._async_result.wait()

        asyncio.run_coroutine_threadsafe(func(), srp_unit.loop).result(200)


if __name__ == '__main__':
    unittest.main()  # 运行所有的测试用例
