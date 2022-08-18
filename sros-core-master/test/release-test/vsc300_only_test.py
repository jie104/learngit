#!/usr/bin/env python
# -*- coding:utf-8 -*-
# file vsc300_only_test.py
# author pengjiali
# date 19-7-27.
# copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# describe Test for vsc300 only，you must set the paraments 'debug.enable_sros_native_debug' to be true.

import unittest
import sys

sys.path.append('../communication')
from async_result import AsyncResult, AsyncResultState
import asyncio
import path
from srp import SRP
import time
from srp_unit import SrpUnit


class TestVSC300Only(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        unittest.TestCase.__init__(self, *args, **kwargs)
        self.srp_unit = SrpUnit('192.168.83.201')
        self.srp = self.srp_unit.srp

    def test_cancel_action(self):
        self.srp.excute_action_task(77, 129, 10, 0)
        time.sleep(3)
        self.srp.cancel_action_task()

    def test_get_config(self):
        config = self.srp.get_config('hmi.speaker_volume')
        volume = int(config['value'])
        self.assertLessEqual(volume, 100)
        self.assertGreaterEqual(volume, 0)

    def test_set_config(self):
        volume_expecation = 88
        configs = [{
            'key': 'hmi.speaker_volume',
            'value': str(volume_expecation)
        }]
        self.srp.set_config(configs)
        config = self.srp.get_config('hmi.speaker_volume')
        volume = int(config['value'])
        self.assertEqual(volume, volume_expecation)

    def test_backup_restore_config(self):
        volume_key = 'hmi.speaker_volume'

        config = self.srp.get_config(volume_key)
        old_volume = int(config['value'])

        self.srp_unit.backup_config(volume_key)

        configs = [{
            'key': 'hmi.speaker_volume',
            'value': str(99 if old_volume == 0 else old_volume - 1)
        }]
        self.srp.set_config(configs)

        self.srp_unit.restore_config(volume_key)

        config = self.srp.get_config(volume_key)
        volume = int(config['value'])

        self.assertEqual(volume, old_volume)


if __name__ == '__main__':
    unittest.main()  # 运行所有的测试用例
