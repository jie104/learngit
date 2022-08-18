#!/usr/bin/env python3.7
# -*- coding:utf-8 -*-
# @File: eac_v2_modbus_tcp_simulate.py
# @Author: pengjiali
# @Date: 19-11-14
# @Copyright: Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# @Describe: 本例程展示了如何实现斯坦德AGV动作控制器动作，用户可以自由修改使用本代码
#
# 使用说明：
# 本动作控制器通信方式为modbus-TCP，为slave模式，启动后会监听5020端口号。
# 输入寄存器和保持寄存器的起始地址都是0
# 支持功能请参见各个ActionXXX类的相关注释
#
# 当用sros来调试本例程时需要将sros如下设置：
#   保证sros能访问到本例程的IP，端口号
#   sros版本大于等于v4.9.0，或者是feature/eac分支
#   开启扩展动作控制器：device.enable_eac → True
#   选择扩展动作控制器的通信类型：device.eac_communication_type → modbus_tcp
#   设置扩展动作控制器的IP地址：device.eac_ip
#   设置扩展动作控制器的端口号：device.eac_port -> 5020
#   设置扩展动作控制器的输入寄存器起始地址：device.eac_basis_input_register_addr -> 0
#   设置扩展动作控制器的保持寄存器起始地址：device.eac_basis_hold_register_addr -> 0
#   设置扩展动作控制器slave id：device.eac_slave_id -> 1
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


class SystemState(Enum):
    """系统状态枚举"""
    NONE = 0x00
    INIT = 0x01
    IDLE = 0x02
    RUNNING = 0x03
    ERROR = 0x04


class ActionState(Enum):
    NONE = 0  # 状态不可用
    WAIT_FOR_START = 2  # 等待开始执行
    RUNNING = 3  # 任务正在执行
    PAUSED = 4  # 暂停执行
    FINISHED = 5  # 执行结束
    IN_CANCEL = 6  # 正在取消中


class ActionResult(Enum):
    NONE = 0  # 结果状态不可用
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


class DriveState(Enum):
    DRIVE_STATE_STOP = 1
    DRIVE_STATE_RUN = 2


class SrosActionControl(Enum):
    NONE = 0
    CLEAN = 1
    START = 2
    PAUSE = 3
    CONTINUE = 4
    CANCEL = 5


class ActionHoldRegisterAddr(Enum):
    ACTION_DRIVE_STATE = 0
    RESET = 1
    CLEAR = 2
    CANCEL_ACTION = 3
    EMERGENCY = 4
    PAUSE = 5
    CONTINUE = 6

    NEW_ACTION = 15
    ACTION_NO = 16
    ACTION_ID = 18
    ACTION_PARAM_0 = 20
    ACTION_PARAM_1 = 22

    SROS_ACTION_STATE = 40
    SROS_ACTION_RESULT = 41
    SROS_ACTION_RESULT_VALUE = 42


class ActionInputRegisterAddr(Enum):
    SYSTEM_STATE = 0
    SYSTEM_ERROR_CODE = 1
    ACTION_CONTROLLER_VERSION = 3
    HEART_BEAT = 5
    ACTION_CONTROLLER_TYPE = 7
    MULTI_LOAD_STATE = 8

    NEW_ACTION_RESULT = 10
    ACTION_STATE = 11
    PROHIBIT_MOVEMENT_STATE = 12
    STATE_ACTION_NO = 16
    STATE_ACTION_ID = 18
    STATE_ACTION_PARAM_0 = 20
    STATE_ACTION_PARAM_1 = 22
    ACTION_RESULT = 24
    ACTION_RESULT_VALUE = 25

    DEBUG_REGISTER = 30

    SROS_ACTION_CONTROL = 40
    SROS_ACTION_ID = 41
    SROS_ACTION_PARAM_0 = 43
    SROS_ACTION_PARAM_1 = 45
    SROS_ACTION_PARAM_2 = 47


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


class HardwareState:
    """
    硬件状态，实际应用中必须上传传感器实时的状态，此处为了好演示是维护了一个逻辑状态
    """

    def __init__(self):
        self.multi_load_state = 0  # 有货没货的状态

class ProhibitMovementState:

    def __init__(self):
        self.prohibit_movement_state = 0  # 有货没货的状态


class DebugRegisters:
    """
    10个调试寄存器的值
    """

    def __init__(self):
        self.debug_registers = [0] * 10

    def setRandomValues(self):
        self.debug_registers = random.sample(range(0, 255), 10)


class Action:
    """
    所有动作的基类
    """
    def __init__(self, id, param_0, param_1):
        self.id = id
        self.param_0 = param_0
        self.param_1 = param_1
        self.action_state = ActionState.NONE
        self.action_result = ActionResult.NONE
        self.action_result_value = 0
        self.hardware_state = None
        self.prohibit_movement_state = None
        # sros 子任务相关，不需要用的就不要关心
        self.sros_action_state = ActionState.NONE
        self.sros_action_result = ActionResult.NONE
        self.sros_action_result_value = 0
        self.sros_action_control = SrosActionControl.NONE
        self.sros_action_id = 0
        self.sros_action_param_0 = 0
        self.sros_action_param_1 = 0
        self.sros_action_param_2 = 0

    def set_hardware_state(self, hardware_state):
        self.hardware_state = hardware_state

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
        self.action_state = ActionState.FINISHED
        self.action_result = ActionResult.OK
        self.action_result_value = result_value
        logging.info('action(%d, %d, %d) finish succeed with result value %d.' % (
            self.id, self.param_0, self.param_1, result_value))

    def _finish_failed(self, error_code):
        """
        设置异常结束的状态
        :param error_code: 错误码
        :return:
        """
        self.action_state = ActionState.FINISHED
        self.action_result = ActionResult.FAILED
        self.action_result_value = error_code
        logging.info('action(%d, %d, %d) finish failed with error code %d.' % (
            self.id, self.param_0, self.param_1, error_code))

    def _finish_canceled(self):
        """
        设置任务被取消的状态
        :return:
        """
        self.action_state = ActionState.FINISHED
        self.action_result = ActionResult.CANCELED
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
        self.action_state = ActionState.IN_CANCEL

    def run(self, tick):
        if tick == 5:
            if self.param_0 == 1:
                self.hardware_state.multi_load_state = 1
            else:
                self.hardware_state.multi_load_state = 0
            self._finish_succeed(200)

        if self.action_state == ActionState.IN_CANCEL:
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
        if tick == 5:
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

    def run(self, tick):
        if tick == 1:
            # 清除上一次任务可能遗留的状态
            self.sros_action_control = SrosActionControl.CLEAN
        elif tick == 5:
            # 执行过程中发现当前AGV位置不满足需求，需要将车辆向前平移50mm
            logging.info("Current pose art not met, we need agv to move forward 50mm! Start SROS sub action.")
            self.action_step = 2
            if not self.start_sros_sub_action(40, 500, 0, 0):
                self._finish_failed(19602)
            return

        if self.action_step == 2:
            # 先查看上一次任务的状态
            if self.sros_action_state == ActionState.FINISHED:
                if self.sros_action_result == ActionResult.OK:
                    logging.info("Move forward 50mm finish ok!")
                    # 第二步正常结束，开始执行第三步
                    self.action_step = 3
                    self.tick_counter = 0
                    # 清除执行过的程序状态
                    self.sros_action_control = SrosActionControl.CLEAN
                else:
                    logging.error("finish failed! result is " + str(self.sros_action_result))
                    self._finish_failed(19603)
                    return
        elif self.action_step == 3:
            self.tick_counter += 1
            if self.tick_counter == 2:
                logging.info("Current pose art not met, we need agv to rotate 45°! Start SROS sub action.")
                self.action_step = 4
                if not self.start_sros_sub_action(40, 0, 0, 450):
                    self._finish_failed(19602)
                return
        elif self.action_step == 4:
            # 先查看上一次任务的状态
            if self.sros_action_state == ActionState.FINISHED:
                if self.sros_action_result == ActionResult.OK:
                    logging.info("rotate 45° finish ok!")
                    # 第四步正常结束，开始执行第五步
                    self.action_step = 5
                    self.tick_counter = 0
                    # 清除执行过的程序状态
                    self.sros_action_control = SrosActionControl.CLEAN
                else:
                    logging.error("finish failed! result is " + str(self.sros_action_result))
                    self._finish_failed(19603)
                    return
        elif self.action_step == 5:
            self.tick_counter += 1
            if self.tick_counter == 5:
                self._finish_succeed(100)

    def start_sros_sub_action(self, id, param_0, param_1, param_2) -> bool:
        """
        启动sros子任务
        :param id:
        :param param_0:
        :param param_1:
        :param param_2:
        :return: True -- 启动成功； False -- 启动失败
        """
        if self.sros_action_state != ActionState.NONE:
            logging.error("Sros not ready for new sub action! sros_action_state is %s." % self.sros_action_state)
            return False

        self.sros_action_control = SrosActionControl.START
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
        if self.action_state == ActionState.FINISHED:
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
        old_reset = self.store['h'].getValues(ActionHoldRegisterAddr.RESET.value)
        old_clear = self.store['h'].getValues(ActionHoldRegisterAddr.CLEAR.value)
        old_cancel_action = self.store['h'].getValues(ActionHoldRegisterAddr.CANCEL_ACTION.value)
        old_emergency = self.store['h'].getValues(ActionHoldRegisterAddr.EMERGENCY.value)
        old_pause = self.store['h'].getValues(ActionHoldRegisterAddr.PAUSE.value)
        old_continue = self.store['h'].getValues(ActionHoldRegisterAddr.CONTINUE.value)
        old_new_action = self.store['h'].getValues(ActionHoldRegisterAddr.NEW_ACTION.value)

        ModbusSlaveContext.setValues(self, fx, address, values)

        new_reset = self.store['h'].getValues(ActionHoldRegisterAddr.RESET.value)
        new_clear = self.store['h'].getValues(ActionHoldRegisterAddr.CLEAR.value)
        new_cancel_action = self.store['h'].getValues(ActionHoldRegisterAddr.CANCEL_ACTION.value)
        new_emergency = self.store['h'].getValues(ActionHoldRegisterAddr.EMERGENCY.value)
        new_pause = self.store['h'].getValues(ActionHoldRegisterAddr.PAUSE.value)
        new_continue = self.store['h'].getValues(ActionHoldRegisterAddr.CONTINUE.value)
        new_new_action = self.store['h'].getValues(ActionHoldRegisterAddr.NEW_ACTION.value)

        if old_reset != new_reset and new_reset[0] == 0x01:
            self._reset()

        if old_clear != new_clear and new_clear[0] == 0x01:
            self._clear()

        if old_cancel_action != new_cancel_action and new_cancel_action[0] == 0x01:
            self._cancel_action()

        if old_emergency != new_emergency:
            self._set_emergency(True if new_emergency[0] == 0x01 else False)

        if old_pause != new_pause and new_pause[0] == 0x01:
            self._pause()

        if old_continue != new_continue and new_continue[0] == 0x01:
            self._continue()

        if old_new_action != new_new_action and new_new_action[0] != 0:
            self._start_new_action()

        if self._system_state == SystemState.RUNNING:
            self._action.sros_action_state = ActionState(
                self.store['h'].getValues(ActionHoldRegisterAddr.SROS_ACTION_STATE.value)[
                    0])
            self._action.sros_action_result = \
                ActionResult(self.store['h'].getValues(ActionHoldRegisterAddr.SROS_ACTION_RESULT.value)[0])
            self._action.sros_action_result_value = double_uint16_to_uint32(
                self.store['h'].getValues(ActionHoldRegisterAddr.SROS_ACTION_RESULT_VALUE.value, 2))

    def _reset(self):
        logging.info('reset')
        self._system_state = SystemState.IDLE
        self._error_code = 0
        self._version = 2002000
        self._heart_beat = int(time.time())
        self._action_controller_type = 192
        self._new_action_result = NewActionResult.NONE
        self._action_no = 0
        self._emergency = False  # 是否处于急停状态
        self._hardware_state = HardwareState()
        self._prohibit_movement_state = ProhibitMovementState()
        self._action = Action(0, 0, 0)
        self._debug_registers = DebugRegisters()
        self._update_input_registers()

    def _clear(self):
        logging.info('clear')
        self._new_action_result = NewActionResult.NONE
        self._update_input_registers()

    def _cancel_action(self):
        logging.info('cancel action')
        self._action.cancel()
        self._update_input_registers()

    def _set_emergency(self, enable):
        logging.info('set emergency %s' % enable)
        self._emergency = enable

    def _pause(self):
        logging.info('pause')
        if self._system_state == SystemState.RUNNING and self._action.action_state == ActionState.RUNNING:
            self._action.action_state = ActionState.PAUSED
            self._update_input_registers()

    def _continue(self):
        logging.info('continue')
        if self._system_state == SystemState.RUNNING and self._action.action_state == ActionState.PAUSED:
            self._action.action_state = ActionState.RUNNING
            self._update_input_registers()

    def _update_heart_beat(self):
        while True:
            time.sleep(0.1)
            self._heart_beat = int(time.time_ns() / 1000)
            self.store['i'].setValues(ActionInputRegisterAddr.HEART_BEAT.value,
                                      uint32_to_double_uint16(self._heart_beat))

    def _start_new_action(self):
        self._action_no = double_uint16_to_uint32(
            self.store['h'].getValues(ActionHoldRegisterAddr.ACTION_NO.value, 2))
        action_id = double_uint16_to_uint32(
            self.store['h'].getValues(ActionHoldRegisterAddr.ACTION_ID.value, 2))
        action_p0 = double_uint16_to_uint32(
            self.store['h'].getValues(ActionHoldRegisterAddr.ACTION_PARAM_0.value, 2))
        action_p1 = double_uint16_to_uint32(
            self.store['h'].getValues(ActionHoldRegisterAddr.ACTION_PARAM_1.value, 2))
        self._new_action_result = NewActionResult.NONE

        action_class = "Action" + str(action_id)
        if action_class not in globals().keys():
            self._new_action_result = NewActionResult.ACTION_ID_NOT_SUPPORT
            logging.error('action start failed! id' + str(action_id) + 'not support!')
        else:
            self._action = globals()[action_class](action_p0, action_p1)  # 根据ID创建对应的动作
            self._action.set_hardware_state(self._hardware_state)
            self._action.set_prohibit_movement_state(self._prohibit_movement_state)
            if not self._action.is_param_0_legal():
                self._new_action_result = NewActionResult.ACTION_PARAM_0_NOT_SUPPORT
                logging.error('Action start failed! param 0 %d not support!' % self._action.param_0)
            elif not self._action.is_param_1_legal():
                self._new_action_result = NewActionResult.ACTION_PARAM_1_NOT_SUPPORT
                logging.error('Action start failed! param 1 %d not support!' % self._action.param_0)
            else:
                logging.info('Start action %d (%d, %d, %d)' % (
                    self._action_no, self._action.id, self._action.param_0, self._action.param_1))
                self._new_action_result = NewActionResult.SUCCEED
                self._action.action_state = ActionState.RUNNING
                self._system_state = SystemState.RUNNING
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
            if self._system_state == SystemState.RUNNING:
                if self._emergency:
                    logging.info('Action in running and in emergency!, tick: %d' % tick)
                    continue

                if self._action.action_state == ActionState.PAUSED:
                    logging.info('action in running and in paused!, tick: %d' % tick)
                    continue

                tick += 1
                logging.info('action in running, tick: %d' % tick)
                try:
                    self._action.run(tick)
                    if self._action.action_state == ActionState.FINISHED:
                        self._system_state = SystemState.IDLE
                    self._update_input_registers()
                except HardwareException as e:
                    logging.error(e)
                    self._system_state = SystemState.ERROR
                    self._error_code = e.hardware_error.value
                    break
            else:
                self._system_state = SystemState.IDLE
                break  # 其他状态就结束任务运行
        self._update_input_registers()

    def _update_input_registers(self):
        """保证每一次同步所有的寄存器"""
        self.store['i'].setValues(ActionInputRegisterAddr.SYSTEM_STATE.value, self._system_state.value)
        self.store['i'].setValues(ActionInputRegisterAddr.SYSTEM_ERROR_CODE.value,
                                  uint32_to_double_uint16(self._error_code))
        self.store['i'].setValues(ActionInputRegisterAddr.ACTION_CONTROLLER_VERSION.value,
                                  uint32_to_double_uint16(self._version))
        self.store['i'].setValues(ActionInputRegisterAddr.HEART_BEAT.value,
                                  uint32_to_double_uint16(self._heart_beat))
        self.store['i'].setValues(ActionInputRegisterAddr.ACTION_CONTROLLER_TYPE.value, self._action_controller_type)
        self.store['i'].setValues(ActionInputRegisterAddr.MULTI_LOAD_STATE.value, self._hardware_state.multi_load_state)
        self.store['i'].setValues(ActionInputRegisterAddr.NEW_ACTION_RESULT.value, self._new_action_result.value)
        self.store['i'].setValues(ActionInputRegisterAddr.ACTION_STATE.value, self._action.action_state.value)
        print('禁止移动：',self._prohibit_movement_state.prohibit_movement_state)
        self.store['i'].setValues(ActionInputRegisterAddr.PROHIBIT_MOVEMENT_STATE.value,
                                  self._prohibit_movement_state.prohibit_movement_state)

        self.store['i'].setValues(ActionInputRegisterAddr.STATE_ACTION_NO.value,
                                  uint32_to_double_uint16(self._action_no))
        self.store['i'].setValues(ActionInputRegisterAddr.STATE_ACTION_ID.value,
                                  uint32_to_double_uint16(self._action.id))
        self.store['i'].setValues(ActionInputRegisterAddr.STATE_ACTION_PARAM_0.value,
                                  uint32_to_double_uint16(self._action.param_0))
        self.store['i'].setValues(ActionInputRegisterAddr.STATE_ACTION_PARAM_1.value,
                                  uint32_to_double_uint16(self._action.param_1))
        self.store['i'].setValues(ActionInputRegisterAddr.ACTION_RESULT.value, self._action.action_result.value)
        self.store['i'].setValues(ActionInputRegisterAddr.ACTION_RESULT_VALUE.value,
                                  uint32_to_double_uint16(self._action.action_result_value))

        self.store['i'].setValues(ActionInputRegisterAddr.DEBUG_REGISTER.value, self._debug_registers.debug_registers)

        self.store['i'].setValues(ActionInputRegisterAddr.SROS_ACTION_CONTROL.value,
                                  self._action.sros_action_control.value)
        self.store['i'].setValues(ActionInputRegisterAddr.SROS_ACTION_ID.value,
                                  uint32_to_double_uint16(self._action.sros_action_id))
        self.store['i'].setValues(ActionInputRegisterAddr.SROS_ACTION_PARAM_0.value,
                                  uint32_to_double_uint16(self._action.sros_action_param_0))
        self.store['i'].setValues(ActionInputRegisterAddr.SROS_ACTION_PARAM_1.value,
                                  uint32_to_double_uint16(self._action.sros_action_param_1))
        self.store['i'].setValues(ActionInputRegisterAddr.SROS_ACTION_PARAM_2.value,
                                  uint32_to_double_uint16(self._action.sros_action_param_2))


class EAC(object):
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
        logging.info('run')
        if modbus_type == ModbusType.MODBUS_TCP:
            StartTcpServer(self.__context, address=("0.0.0.0", 5020))
        elif modbus_type == ModbusType.MODBUS_RTU:
            StartSerialServer(self.__context, framer=ModbusRtuFramer, port="/dev/ttyUSB0", timeout=.05, baudrate=115200)


if __name__ == '__main__':
    eac = EAC()
    eac.run(ModbusType.MODBUS_TCP)  # 启动Modbus-TCP
    # eac.run(ModbusType.MODBUS_RTU) # 启动Modbus-RTU
