#!/usr/bin/env python
# -*- coding:utf-8 -*-
# file test_pause_level.py
# author lhx
# date 19-11-9.
# copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# describe 在执行路径过程中定时自动插入暂停和继续，测试是否会出现发送暂停而车不停止运动的问题

import sys
from time import sleep

sys.path.append('./communication')
from srp import SRP
import main_pb2

cur_move_task = {}
sys_state = {}


def system_state_callback(data):
    global cur_move_task, sys_state
    sys_state = data
    cur_move_task = data.movement_state
    # print(cur_move_task)
    pass


if __name__ == '__main__':
    srp = SRP()
    srp.set_system_state_callback(system_state_callback)
    try:
        srp.login('192.168.80.212', 'admin', 'admin')
        # srp.triger_emergency()
        # srp.move_to_station(33, 116)
    except BaseException as e:
        print(e)

    sleep(2)

    stations = [34, 116]

    for i in range(1, 100):
        print("-------------------------")
        print("loop ", i)

        srp.move_to_station(i, stations[i % len(stations)])

        # print(cur_move_task)

        while cur_move_task.no != i:
            print("waiting for cur move task no from ", cur_move_task.no, " => ", i)
            sleep(0.1)

        sleep(2)

        print(cur_move_task.state)
        pause_level = 0
        while cur_move_task.state != main_pb2.MovementTask.MT_FINISHED:
            # print("running")
            pause_level = (pause_level + 1) % 5

            srp.pause_movement(pause_level)

            sleep(2)

            v_x = sys_state.mc_state.v_x
            print("running, pause level ", pause_level, " speed is ", v_x)
            if v_x > 0.05:
                print("===================> FAIL")
                exit(-1)

            srp.continue_movement()

            sleep(3)

        print("done")

        sleep(2)

        # srp.move_to_station(i + 1, 34)



