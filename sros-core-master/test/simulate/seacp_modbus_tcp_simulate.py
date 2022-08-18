#!/usr/bin/env python3.7
# -*- coding:utf-8 -*-
# @File: seacp_v2_modbus_tcp_simulate.py
# @Author: caoyan
# @Date: 2021-07-07
# @Copyright: Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# @Describe: 本例程展示了如何实现斯坦德AGV动作控制器动作，用户可以自由修改使用本代码
#
# 使用说明：
# 本动作控制器通信方式为modbus-TCP，为slave模式，启动后会监听5020端口号。
# 输入寄存器和保持寄存器的起始地址都是0
# 支持功能请参见各个ActionXXX类的相关注释
#
#   若您用的是串口：
#   设置串口名：device.eac_serial_device -> 串口名，主要不要串口冲突了，若冲突了EAC会被禁用，系统日志中能显示是否冲突
#   设置串口波特率：device.eac_serial_baud_rate
#   串口数据位固定为8位，停止位固定为1位，无奇偶校验

from pymodbus.server.sync import StartTcpServer, StartSerialServer
from pymodbus.datastore import ModbusSlaveContext, ModbusSequentialDataBlock
from pymodbus.constants import Defaults
from pymodbus.datastore import ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer
from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder
from pymodbus.constants import Endian
import sys
import logging
from enum import Enum
import time
import threading
import random

Defaults.ZeroMode = True

FORMAT = ('%(asctime)-15s %(threadName)-15s'
          ' %(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
logging.basicConfig(stream=sys.stdout, level=logging.INFO, format=FORMAT)


class EacSysControl(Enum):
    """动作控制"""
    NONE = 0x00
    RESET = 0x01
    CLEAR = 0x02


class EacActControl(Enum):
    """动作控制"""
    NONE = 0x00
    START = 0x01
    PAUSE = 0x02
    RUNNING = 0x03
    CANCEL = 0x04


class EacSysState(Enum):
    """系统状态枚举"""
    NONE = 0x00
    INIT = 0x01
    ERROR = 0x02
    RUNNING = 0x03


class EacActState(Enum):
    NONE = 0x00  # 无效状态
    IDLE = 0x01
    RUNNING = 0x02  # 任务正在执行
    PAUSED = 0x03  # 暂停中
    ERROR_PAUSED = 0x04  # 异常暂停中
    FINISHED = 0x05  # 执行结束
    FAILED = 0x06
    CANCEL = 0x07  # 执行取消


class LoadState(Enum):
    NONE = 0x00
    LOAD = 0x01


class AssistFunc(Enum):
    NONE = 0X00
    PROHIBIT_MOVEMENT = 0X01


# 扩展功能
class SrosActState(Enum):
    NONE = 0x00  # 无效状态
    IDLE = 0x01
    RUNNING = 0x02  # 任务正在执行
    PAUSED = 0x03  # 暂停中
    FINISHED = 0x04  # 执行结束
    FAILED = 0x05
    CANCEL = 0x06  # 执行取消


class SrosSysControl(Enum):
    NONE = 0X00
    CLEAR = 0X01


class SrosActControl(Enum):
    NONE = 0X00
    START = 0X01
    PAUSE = 0X02
    CONTINUE = 0X03
    CANCEL = 0X04


# 寄存器地址
class ActionHoldRegisterAddr(Enum):
    EAC_SYS_CONTROL = 0
    EAC_ACT_CONTROL = 1
    ACTION_NO = 2
    ACTION_ID = 4
    ACTION_PARAM_0 = 5
    ACTION_PARAM_1 = 6
    # 保持扩展偏移10
    SROS_ACT_STATE = 10
    SROS_ACT_RESULT = 11
    SROS_ACTION_NO = 13


class ActionInputRegisterAddr(Enum):
    EAC_SYS_STATE = 0
    EAC_SYS_ERROR_CODE = 1
    EAC_ACT_STATE = 2
    EAC_ACT_RESULT = 3
    RUN_ACTION_NO = 5
    LOAD_STATE = 7
    ASSIST_FUNC = 8
    HEART_BEAT = 9
    EAC_SYS_VERSION = 10
    # 输入扩展偏移20
    SROS_SYS_CONTROL = 20
    SROS_ACT_CONTROL = 21
    SROS_ACTION_NO = 22
    SROS_ACTION_ID = 24
    SROS_ACTION_PARAM_0 = 25
    SROS_ACTION_PARAM_1 = 26
    SROS_ACTION_PARAM_2 = 27


class ModbusType(Enum):
    MODBUS_TCP = 0
    MODBUS_TCP_PI = 1
    MODBUS_RTU = 2


class HardwareError(Enum):
    """硬件故障枚举"""
    MOTOR_DISCONNECT = 404


class HardwareException(Exception):
    """Hardware exception"""

    def __init__(self, hardware_error):
        self.hardware_error = hardware_error

    def __str__(self):
        return "Hardware Error: " + str(self.hardware_error)


class MultiLoadState:
    """
    硬件状态，实际应用中必须上传传感器实时的状态，此处为了好演示是维护了一个逻辑状态
    """

    def __init__(self):
        self.multi_load_state = LoadState.NONE  # 没货的状态


class ProhibitMovementState:

    def __init__(self):
        self.prohibit_movement_state = AssistFunc.NONE  # 有货没货的状态


class Action:
    """
    所有动作的基类
    """

    def __init__(self, id, param_0, param_1):
        self.id = id
        self.param_0 = param_0
        self.param_1 = param_1
        self.action_state = EacActState.IDLE
        self.action_result = 0
        self.load_state = None
        self.prohibit_movement_state = None
        # sros 子任务相关，不需要用的就不要关心
        self.sros_action_state = SrosActState.NONE
        self.sros_action_result = 0
        self.sros_run_action_no = 0
        self.sros_system_control = SrosSysControl.NONE
        self.sros_action_control = SrosActControl.NONE
        self.sros_action_no = 0
        self.sros_action_id = 0
        self.sros_action_param_0 = 0
        self.sros_action_param_1 = 0
        self.sros_action_param_2 = 0

    def set_load_state(self, load_state):
        self.load_state = load_state

    def set_prohibit_movement_state(self, prohibit_movement_state):
        self.prohibit_movement_state = prohibit_movement_state

    def is_param_0_legal(self) -> bool:
        """
        判断参数参数0是否合法, 默认都是合法的
        :return:
        """
        return True

    def is_param_1_legal(self) -> bool:
        return True

    def is_param_2_legal(self) -> bool:
        return True

    def cancel(self):
        """
        默认取消就直接取消了
        实际上需要需要先设置取消状态,然后完成清理工作，最后取消
        :return:
        """
        self._finish_canceled()

    def run(self, tick):
        """
        执行动作，每个时钟周期只处理一次，不然没法响应外部的暂停、继续等指令
        :param tick: 外部计时器
        :return: True -- 当前任务完成； False -- 当前任务失败
        """
        raise Exception("Please overload the run function! action id is " + str(self.id))

    def _finish_succeed(self, result_value=0):
        """
        设置成功结束的状态
        :param result_value: 执行结果
        :return:
        """
        self.action_state = EacActState.FINISHED
        self.action_result = result_value
        logging.info('action(%d, %d, %d) finish succeed with result value %d.' % (
            self.id, self.param_0, self.param_1, result_value))

    def _finish_failed(self, error_code):
        """
        设置异常结束的状态
        :param error_code: 错误码
        :return:
        """
        self.action_state = EacActState.FAILED
        self.action_result = error_code
        logging.info('action(%d, %d, %d) finish failed with error code %d.' % (
            self.id, self.param_0, self.param_1, error_code))

    def _finish_canceled(self):
        """
        设置任务被取消的状态
        :return:
        """
        self.action_state = EacActState.CANCEL
        logging.info('action(%d, %d, %d) finish canceled!' % (
            self.id, self.param_0, self.param_1))


class Action192(Action):
    """
    本例 10s后将会正常结束, 结果为200，支持的参数如下：
    1. (192, 1, 0) -- 模拟顶升货架，顶升完后会变成有货状态
    2. (192, 2, 0) -- 模拟放下货架，放下完后会变成没货状态
    当本例会演示取消时，先设置取消状态，当动作处理完一些取消需要处理的事情后再真正结束
    """

    def __init__(self, param_0, param_1):
        Action.__init__(self, 192, param_0, param_1)
        self.tick_counter = 0

    def is_param_0_legal(self) -> bool:
        return 0 < self.param_0 < 3

    def is_param_1_legal(self) -> bool:
        return 0 == self.param_1

    def cancel(self):
        """先置取消状态"""
        self.action_state = EacActState.CANCEL

    def run(self, tick):
        if tick == 5:
            if self.param_0 == 1:
                self.load_state.multi_load_state = LoadState.LOAD
            else:
                self.load_state.multi_load_state = LoadState.NONE
            self._finish_succeed(200)

        if self.action_state == EacActState.CANCEL:
            self.tick_counter += 1  # 做一些清理工作，此处模拟将self.tick_counter加到2表示清理完了
            if self.tick_counter == 2:
                self._finish_canceled()


class Action193(Action):
    """
    10秒后将会异常结束， 异常码为19301
    """

    def __init__(self, param_0, param_1):
        Action.__init__(self, 193, param_0, param_1)

    def run(self, tick):
        if tick == 50:
            self._finish_failed(19301)


class Action194(Action):
    """
    5s会会出现故障，系统进入故障状态，需要人工清除故障，故障码404,任务将会异常结束，错误码为19404
    """

    def __init__(self, param_0, param_1):
        Action.__init__(self, 194, param_0, param_1)

    def run(self, tick):
        if tick == 5:
            self._finish_failed(19401)
            raise HardwareException(HardwareError.MOTOR_DISCONNECT)


class Action196(Action):
    """
    动作196是模拟处理SROS子任务的实现，处理步骤如下：
    1. 前5秒EAC自己执行
    2. EAC发现当前位置不满足动作执行要求了，需要将AGV向前移动50mm,并启动SROS子任务
    3. 当第二步执行完后，EAC继续执行自行的2秒
    4. EAC再发发现置不满足动作执行要求了，需要AGV将角度逆时针旋转45°
    5. EAC继续执行5秒
    """

    def __init__(self, param_0, param_1):
        Action.__init__(self, 196, param_0, param_1)
        # 下面是两个内部控制变量，
        # 第一个控制当前执行上述的第几步
        # 第二个控制延时用的
        self.action_step = 1
        self.tick_counter = 0
        self.mark_tick = -1

    def run(self, tick):

        if self.mark_tick == -1 and self.sros_action_state == SrosActState.IDLE:
            if self.start_sros_sub_action(40, 500, 400, 100):
                print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>',self.sros_action_state)
                self.action_step = 2
                print('----------------------------------')
                self.mark_tick = tick

        if self.mark_tick != -1 and tick > self.mark_tick:
            if self.sros_action_state == SrosActState.FINISHED:
                self._finish_succeed(200)
                logging.info("sros action finished!!!!!")
                self.sros_system_control = SrosSysControl.CLEAR
                self.sros_action_control = SrosActControl.NONE
            if self.sros_action_state == SrosActState.FAILED:
                self._finish_failed(19605)
                logging.info("sros action failed!!!!!")
                self.sros_system_control = SrosSysControl.CLEAR
                self.sros_action_control = SrosActControl.NONE
            if self.sros_action_state == SrosActState.IDLE:
                self._finish_failed(19605)
                logging.info("sros action failed!!!!!")
                self.sros_system_control = SrosSysControl.CLEAR
                self.sros_action_control = SrosActControl.NONE
            self.sros_action_no = 0
            self.sros_action_id = 0
            self.sros_action_param_0 = 0
            self.sros_action_param_1 = 0
            self.sros_action_param_2 = 0


    def start_sros_sub_action(self, id, param_0, param_1, param_2) -> bool:
        """
        启动sros子任务
        :param id:
        :param param_0:
        :param param_1:
        :param param_2:
        :return: True -- 启动成功； False -- 启动失败
        """
        if self.sros_action_state != SrosActState.IDLE:
            logging.error("Sros not ready for new sub action! sros_action_state is %s." % self.sros_action_state)
            return False

        self.sros_action_control = SrosActControl.START
        self.sros_action_id = id
        self.sros_action_param_0 = param_0
        self.sros_action_param_1 = param_1
        self.sros_action_param_2 = param_2
        return True


class Action197(Action):
    """
    动作197是解除执行机构动作禁止移动。
    """

    def __init__(self, param_0, param_1):
        Action.__init__(self, 197, param_0, param_1)

    def is_param_0_legal(self) -> bool:
        return 0 < self.param_0 < 3

    def is_param_1_legal(self) -> bool:
        return 0 == self.param_1

    def run(self, tick):
        self._finish_succeed(100)
        if self.action_state == EacActState.FINISHED:
            if self.param_0 == 1:
                self.prohibit_movement_state.prohibit_movement_state = 1
                logging.info("Prohibit movement.")
            elif self.param_0 == 2:
                self.prohibit_movement_state.prohibit_movement_state = 0
                logging.info("Lift the ban on movement.")


def double_uint16_to_uint32(double_uint16):
    """
    两个16位的寄存器拼成一个32位的
    :param double_uint16:
    :return:
    """
    num = double_uint16[0] << 16
    num += double_uint16[1]
    return num


def uint32_to_double_uint16(uint32):
    """
    一个32位寄存器拆分成两个16的寄存器
    :param uint32:
    :return:
    """
    return [(uint32 >> 16) & 0xFFFF, uint32 & 0xFFFF]


class EacContext(ModbusSlaveContext):
    def __init__(self, *args, **kwargs):
        ModbusSlaveContext.__init__(self, *args, **kwargs)

        self._reset()

        t = threading.Thread(target=EacContext._update_heart_beat, args=(self,))
        t.start()

    def setValues(self, fx, address, values):
        # print(fx, address, values)
        old_system_control = self.store['h'].getValues(0)
        old_action_control = self.store['h'].getValues(1)
        ModbusSlaveContext.setValues(self, fx, address, values)
        new_system_control = self.store['h'].getValues(0)
        new_action_control = self.store['h'].getValues(1)

        if old_system_control != new_system_control:
            if new_system_control[0] == 0x01:
                self._reset()
            elif new_system_control[0] == 0x02:
                self._clear()

        if old_action_control != new_action_control:
            if new_action_control[0] == 0x01:
                self._start_new_action()
            elif new_action_control[0] == 0x02:
                self._pause()
            elif new_action_control[0] == 0x03:
                self._continue()
            elif new_action_control[0] == 0x04:
                self._cancel_action()

        if self._system_state == EacSysState.RUNNING:
            self._action.sros_action_state = SrosActState(
                self.store['h'].getValues(ActionHoldRegisterAddr.SROS_ACT_STATE.value)[
                    0])
            self._action.sros_action_result = self.store['h'].getValues(ActionHoldRegisterAddr.SROS_ACT_RESULT.value)[0]
            self._action.sros_action_id = self.store['h'].getValues(ActionHoldRegisterAddr.SROS_ACTION_NO.value)[0]
            # print('sros_action_state : ',self._action.sros_action_state)
    def _reset(self):
        logging.info('reset')
        self._system_state = EacSysState.RUNNING
        self._error_code = 0
        self._version = 20020
        self._heart_beat = 0
        self._action_controller_type = 192
        self._action_no = 0
        self._emergency = False  # 是否处于急停状态
        self._load_state = MultiLoadState()
        self._prohibit_movement_state = ProhibitMovementState()
        self._action = Action(0, 0, 0)
        self._update_input_registers()

    def _clear(self):
        logging.info('clear')
        self._action.action_state = EacActState.IDLE
        self._action.action_result = 0
        self._action_no = 0
        self._update_input_registers()

    def _cancel_action(self):
        logging.info('cancel action')
        self._action.cancel()
        self._update_input_registers()

    def _pause(self):
        logging.info('pause')
        if self._system_state == EacSysState.RUNNING and self._action.action_state == EacActState.RUNNING:
            self._action.action_state = EacActState.PAUSED
            self._update_input_registers()

    def _continue(self):
        logging.info('continue')
        if (self._system_state == EacSysState.RUNNING and self._action.action_state == EacActState.PAUSED) \
                or (self._system_state == EacSysState.RUNNING and self._action.action_state == EacActState.ERROR_PAUSED):
            self._action.action_state = EacActState.RUNNING
            self._update_input_registers()

    def _update_heart_beat(self):
        while True:
            time.sleep(0.1)
            for self._heart_beat in range(65000):
                self.store['i'].setValues(ActionInputRegisterAddr.HEART_BEAT.value, self._heart_beat)
                time.sleep(0.001)

    def _start_new_action(self):
        self._action_no = double_uint16_to_uint32(
            self.store['h'].getValues(ActionHoldRegisterAddr.ACTION_NO.value, 2))
        action_id = self.store['h'].getValues(ActionHoldRegisterAddr.ACTION_ID.value, 1)[0]
        action_p0 = self.store['h'].getValues(ActionHoldRegisterAddr.ACTION_PARAM_0.value, 1)[0]
        action_p1 = self.store['h'].getValues(ActionHoldRegisterAddr.ACTION_PARAM_1.value, 1)[0]
        action_class = "Action" + str(action_id)
        if action_class not in globals().keys():
            self._action.action_state = EacActState.FAILED
            logging.error('action start failed! action id' + str(action_id) + ' not support!')
        else:
            self._action = globals()[action_class](action_p0, action_p1)  # 根据ID创建对应的动作
            self._action.set_load_state(self._load_state)
            self._action.set_prohibit_movement_state(self._prohibit_movement_state)
            if not self._action.is_param_0_legal():
                self._action.action_state = EacActState.FAILED
                logging.error('Action start failed! param0 %d not support!' % self._action.param_0)
            elif not self._action.is_param_1_legal():
                self._action.action_state = EacActState.FAILED
                logging.error('Action start failed! param1 %d not support!' % self._action.param_0)
            else:
                logging.info('Start action %d (%d, %d, %d)' % (
                    self._action_no, self._action.id, self._action.param_0, self._action.param_1))
                if self._system_state == EacSysState.RUNNING:
                    self._action.action_state = EacActState.RUNNING
                else:
                    logging.error('system state error!')
                    return
                t = threading.Thread(target=EacContext._action_running, args=(self,))
                t.start()
        self._update_input_registers()

    def _action_running(self):
        """
        eac 真正的动作执行者，在另一个线程中运行
        :return:
        """
        logging.info('action start running!')
        tick = 0

        while True:
            time.sleep(1)  # NOTE： 此处为了简便，每次sleep一秒，但是响应可能没那么及时
            if self._system_state == EacSysState.ERROR:
                logging.info('system state error!, tick: %d' % tick)
                continue

            if self._action.action_state == EacActState.PAUSED:
                logging.info('action in running and in paused!, tick: %d' % tick)
                continue
            if self._action.action_state == EacActState.ERROR_PAUSED:
                self._system_state = EacSysState.ERROR
                logging.info('action in running and in error paused!, tick: %d' % tick)
                continue

            tick += 1
            logging.info('action in running, tick: %d' % tick)
            try:
                self._action.run(tick)
                if self._action.action_state == EacActState.FINISHED \
                        or self._action.action_state == EacActState.FAILED \
                        or self._action.action_state == EacActState.CANCEL:
                    self._update_input_registers()
                    time.sleep(1)
                    self._action.action_state = EacActState.IDLE
                    self._update_input_registers()
                    break
                self._update_input_registers()
            except HardwareException as e:
                logging.error(e)
                self._system_state = EacSysState.ERROR
                self._error_code = e.hardware_error.value
                break

        time.sleep(1)
        self._action.sros_system_control = SrosSysControl.NONE
        self._update_input_registers()

    def _update_input_registers(self):
        """保证每一次同步所有的寄存器"""
        self.store['i'].setValues(ActionInputRegisterAddr.EAC_SYS_STATE.value, self._system_state.value)
        self.store['i'].setValues(ActionInputRegisterAddr.EAC_SYS_ERROR_CODE.value, self._error_code)
        self.store['i'].setValues(ActionInputRegisterAddr.EAC_ACT_STATE.value, self._action.action_state.value)
        self.store['i'].setValues(ActionInputRegisterAddr.EAC_ACT_RESULT.value,
                                  uint32_to_double_uint16(self._action.action_result))
        self.store['i'].setValues(ActionInputRegisterAddr.RUN_ACTION_NO.value,
                                  uint32_to_double_uint16(self._action_no))
        self.store['i'].setValues(ActionInputRegisterAddr.LOAD_STATE.value, self._load_state.multi_load_state.value)
        self.store['i'].setValues(ActionInputRegisterAddr.ASSIST_FUNC.value,
                                  self._prohibit_movement_state.prohibit_movement_state.value)
        self.store['i'].setValues(ActionInputRegisterAddr.HEART_BEAT.value, self._heart_beat)
        self.store['i'].setValues(ActionInputRegisterAddr.EAC_SYS_VERSION.value, self._version)

        # 扩展动作
        self.store['i'].setValues(ActionInputRegisterAddr.SROS_SYS_CONTROL.value, self._action.sros_system_control.value)
        self.store['i'].setValues(ActionInputRegisterAddr.SROS_ACT_CONTROL.value,
                                  self._action.sros_action_control.value)
        self.store['i'].setValues(ActionInputRegisterAddr.SROS_ACTION_NO.value, uint32_to_double_uint16(self._action.sros_action_no))
        self.store['i'].setValues(ActionInputRegisterAddr.SROS_ACTION_ID.value, self._action.sros_action_id)
        self.store['i'].setValues(ActionInputRegisterAddr.SROS_ACTION_PARAM_0.value, self._action.sros_action_param_0)
        self.store['i'].setValues(ActionInputRegisterAddr.SROS_ACTION_PARAM_1.value, self._action.sros_action_param_1)
        self.store['i'].setValues(ActionInputRegisterAddr.SROS_ACTION_PARAM_2.value, self._action.sros_action_param_2)


class SEACP(object):
    def __init__(self):
        store = {
            'di': ModbusSequentialDataBlock(0, [0] * 10),
            'co': ModbusSequentialDataBlock(0, [0] * 10),
            'ir': ModbusSequentialDataBlock(0, [0] * 100),
            'hr': ModbusSequentialDataBlock(0, [0] * 100),
        }
        slave = EacContext(**store)
        self.__context = ModbusServerContext(slaves=slave, single=True)

    def run(self, modbus_type):
        logging.info('seacp run')
        if modbus_type == ModbusType.MODBUS_TCP:
            StartTcpServer(self.__context, address=("0.0.0.0", 5022))
        elif modbus_type == ModbusType.MODBUS_RTU:
            StartSerialServer(self.__context, framer=ModbusRtuFramer, port="/dev/ttyUSB0", timeout=.05, baudrate=115200)


if __name__ == '__main__':
    seacp = SEACP()
    seacp.run(ModbusType.MODBUS_TCP)  # 启动Modbus-TCP
    # eac.run(ModbusType.MODBUS_RTU) # 启动Modbus-RTU
