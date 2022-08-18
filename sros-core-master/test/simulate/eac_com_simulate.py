#!/usr/bin/python3
# -*- coding:utf-8 -*-
# Author:pengjiali
"""
模拟EAC串口通信
部分异常没有处理，若出问题了需要重新启动
"""

import serial
from enum import Enum
import threading


class FrameState(Enum):
    """帧状态枚举"""
    F_IDLE = 0
    F_DATA = 1
    F_CKSUM = 2
    F_READY = 3


class SystemState(Enum):
    """系统状态枚举"""
    SYSTEM_STATE_NONE = 0x00
    SYSTEM_STATE_INIT = 0x01
    SYSTEM_STATE_IDLE = 0x02
    SYSTEM_STATE_RUNNING = 0x03
    SYSTEM_STATE_ERROR = 0x04


class ActionState(Enum):
    NA = 0  # 状态不可用
    WAIT_FOR_START = 2  # 等待开始执行
    RUNNING = 3  # 任务正在执行
    PAUSED = 4  # 暂停执行
    FINISHED = 5  # 执行结束
    IN_CANCEL = 6  # 正在取消中


class ActionResult(Enum):
    NA = 0  # 结果状态不可用
    OK = 1  # 任务执行完成
    CANCELED = 2  # 任务取消
    FAILED = 3  # 任务执行出错


class RequestCmd(Enum):
    GET_VERSION = 0x02
    GET_SYSTEM_STATE = 0x04
    RESET = 0x06

    GET_ACTION_STATE = 0x10
    NEW_ACTION = 0x12
    CANCEL_ACTION = 0x14

    GET_DEBUG_INFO = 0x70


class NewActionResult(Enum):
    NONE = 0x00
    SUCCEED = 0x01
    SYSTEM_BUSY = 0x02
    ACTION_ID_NOT_SUPPORT = 0x03
    ACTION_PARAM_0_NOT_SUPPORT = 0x04
    ACTION_PARAM_1_NOT_SUPPORT = 0x05

class CancelActionResult(Enum):
    NONE = 0x00
    SUCCEED = 0x01
    FAILED = 0x02

class Eac:
    def __init__(self):
        self.__RECIVER_START = b'\xff'  # 帧开头标志
        self.__frame_state = FrameState.F_IDLE  # 读取帧状态
        self.__payload = []  # 读到的一帧数据
        self.__num = 0  # 用于改变系统状态
        self.__system_state = SystemState.SYSTEM_STATE_NONE  # 系统状态
        self.__system_error_no = 0  # 系统错误编码
        self.__action_no = 0  # 动作编号
        self.__action_id = 0  # 动作id
        self.__action_state = ActionState.NA  # 动作状态
        self.__action_result = ActionResult.NA  # 动作执行结果
        self.__action_result_value = 0x0000  # 动作执行结果的值
        self.__response_data = []  # 回复的内容
        self.__response_frame = bytes()  # 回复的帧
        self.__is_response_frame_ready = False  # 回复帧是否准备好
        self.__thread_action = None # 动作执行线程

        self.__thread_system_state_change = threading.Timer(1, self.__system_state_change_fun)
        self.__thread_system_state_change.start()

    def set_data(self, c):
        eac.__feed(c)
        if self.__frame_state != FrameState.F_READY:
            return
        self.__frame_state = FrameState.F_IDLE

        print(self.__system_state, self.__action_state, self.__action_result, self.__action_result_value)
        # print('>> ', self.__payload)
        self.__generate_response_data()
        self.__generate_response_frame()
        self.__is_response_frame_ready = True
        # print('<<', self.__response_frame, '\n')

    def is_response_ready(self):
        return self.__is_response_frame_ready

    def get_response_frame(self):
        self.__is_response_frame_ready = False
        return self.__response_frame

    def __system_state_change_fun(self):
        """模拟系统状态变化，2秒后变成正在初始化、10秒后变成空闲.."""
        # print(self.__num)
        self.__num += 1
        if self.__num == 2:
            self.__system_state = SystemState.SYSTEM_STATE_INIT
        elif self.__num == 10:
            self.__system_state = SystemState.SYSTEM_STATE_IDLE
        elif self.__num == 90:
            self.__system_state = SystemState.SYSTEM_STATE_ERROR
            self.__system_error_no = 0x44  # 模拟出错码
        elif self.__num == 100:
            self.__num = 0
            self.__system_error_no = 0  # 清除出错码

        self.__thread_system_state_change = threading.Timer(1, self.__system_state_change_fun)
        self.__thread_system_state_change.start()

    def __feed(self, c):
        """填充一帧数据"""
        if self.__frame_state == FrameState.F_IDLE:
            if c == self.__RECIVER_START:
                self.__payload.clear()
                self.__frame_state = FrameState.F_DATA
        elif self.__frame_state == FrameState.F_DATA:
            self.__payload.append(c)
            if len(self.__payload) == 8:
                self.__frame_state = FrameState.F_CKSUM
        elif self.__frame_state == FrameState.F_CKSUM:
            sum = 0xff
            for data in self.__payload:
                sum += int.from_bytes(data, 'big', signed=False)
            if sum & 0xFF == int.from_bytes(c, 'big', signed=False):
                self.__frame_state = FrameState.F_READY
            else:
                self.__frame_state = FrameState.F_IDLE
                print('校验出错')

    def __reset(self):
        self.__frame_state = FrameState.F_IDLE
        self.__payload = []
        self.__num = 0
        self.__system_state = SystemState.SYSTEM_STATE_NONE
        self.__system_error_no = 0
        self.__action_no = 0  # 动作编号
        self.__action_id = 0
        self.__action_state = ActionState.NA
        self.__action_result = ActionResult.NA
        self.__action_result_value = 0x0000

    def __generate_response_data(self):
        self.__response_data = [0, 0, 0, 0, 0, 0, 0, 0]
        cmd = int.from_bytes(self.__payload[0], 'big', signed=False)
        self.__response_data[0] = cmd + 1
        if cmd == RequestCmd.GET_VERSION.value:
            print('get version')
            # 硬件版本 v3.4.2
            self.__response_data[1] = 0x03
            self.__response_data[2] = 0x04
            self.__response_data[3] = 0x02
            # 软件版本 v4.6.0
            self.__response_data[4] = 0x04
            self.__response_data[5] = 0x06
            self.__response_data[6] = 0x00
        elif cmd == RequestCmd.GET_SYSTEM_STATE.value:
            self.__response_data[1] = self.__system_state.value
            self.__response_data[2] = self.__system_error_no
        elif cmd == RequestCmd.RESET.value:  # reset
            print('reset cmd')
            self.__reset()
        elif cmd == RequestCmd.GET_ACTION_STATE.value:
            self.__response_data[1] = self.__action_no
            self.__response_data[2] = self.__action_id >> 8 & 0xff
            self.__response_data[3] = self.__action_id & 0xff
            self.__response_data[4] = self.__action_state.value
            self.__response_data[5] = self.__action_result.value
            self.__response_data[6] = self.__action_result_value >> 8 & 0xff
            self.__response_data[7] = self.__action_result_value & 0xff
        elif cmd == RequestCmd.NEW_ACTION.value:
            action_no = int.from_bytes(self.__payload[1], 'big', signed=False)
            tmp = self.__payload[2] + self.__payload[3]
            action_id = int.from_bytes(tmp, 'big', signed=True)
            tmp = self.__payload[4] + self.__payload[5]
            action_param_0 = int.from_bytes(tmp, 'big', signed=True)
            tmp = self.__payload[6] + self.__payload[7]
            action_param_1 = int.from_bytes(tmp, 'big', signed=True)
            print('new action: ', action_no, action_id, action_param_0, action_param_1)
            # 只支持动作(192,1,1)执行成功结果为1111  和 (193,2,2)执行失败结果为2222
            if action_id != 192 and action_id != 193:
                self.__response_data[1] = NewActionResult.ACTION_ID_NOT_SUPPORT.value
                return
            if action_param_0 != 1 and action_param_0 != 2:
                self.__response_data[1] = NewActionResult.ACTION_PARAM_0_NOT_SUPPORT.value
                return
            if action_param_1 != 1 and action_param_1 != 2:
                self.__response_data[1] = NewActionResult.ACTION_PARAM_1_NOT_SUPPORT.value
                return
            if self.__system_state != SystemState.SYSTEM_STATE_IDLE:
                self.__response_data[1] = NewActionResult.SYSTEM_BUSY.value
                return

            # 校验成功
            self.__system_state = SystemState.SYSTEM_STATE_RUNNING
            self.__action_no = action_no
            self.__action_id = action_id
            self.__action_state = ActionState.RUNNING
            self.__action_result = ActionResult.NA
            self.__action_result_value = 0x0000
            self.__thread_action = threading.Timer(5, self.__on_action_finished)
            self.__thread_action.start()
            self.__response_data[1] = NewActionResult.SUCCEED.value
        elif cmd == RequestCmd.CANCEL_ACTION.value:
            print('cancle action')
            if self.__system_state == SystemState.SYSTEM_STATE_RUNNING:
                self.__action_state = ActionState.FINISHED
                self.__action_result = ActionResult.CANCELED
                self.__system_state = SystemState.SYSTEM_STATE_IDLE
                self.__response_data[1] = CancelActionResult.SUCCEED.value
            else:
                self.__response_data[1] = CancelActionResult.FAILED.value
        elif cmd == RequestCmd.GET_DEBUG_INFO.value:
            self.__response_data[1] = 1
            self.__response_data[2] = 2
            self.__response_data[3] = 3
            self.__response_data[4] = 4
            self.__response_data[5] = 5
            self.__response_data[6] = 6
            self.__response_data[7] = 7


    def __generate_response_frame(self):
        self.__response_frame = b'\xff'
        self.__response_frame += bytes(self.__response_data)
        sum = 0
        for byte in self.__response_frame:
            sum += byte
        self.__response_frame += (sum & 0xFF).to_bytes(1, 'big', signed=False)

    def __on_action_finished(self):
        if self.__system_state == SystemState.SYSTEM_STATE_RUNNING:
            self.__action_state = ActionState.FINISHED
            if self.__action_id == 192:
                self.__action_result = ActionResult.OK
                self.__action_result_value = 1111
            elif self.__action_id == 193:
                self.__action_result = ActionResult.FAILED
                self.__action_result_value = 2222
            self.__system_state = SystemState.SYSTEM_STATE_IDLE


if __name__ == '__main__':
    with serial.Serial('/dev/ttyUSB0', 115200, timeout=1) as ser:
        eac = Eac()
        while True:
            c = ser.read()
            # print(c)
            eac.set_data(c)
            if eac.is_response_ready():
                ser.write(eac.get_response_frame())
