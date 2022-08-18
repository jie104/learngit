#!/usr/bin/env python
# -*- coding:utf-8 -*-
# file srp_protobuf.py
# author pengjiali
# date 19-6-25.
# copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# describe

import hashlib
import main_pb2
import frame as srp_frame
import logging
from google.protobuf.json_format import MessageToDict, ParseDict

_logger = logging.getLogger(__name__)
print(__name__)


class Pose:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw


class SrpProtobuf:
    '''
    本类处理protobuf协议的解析
    NOET:
    1. seq 不放到本类内部处理的原因，由于上层需要处理同步问题，那么上层需要seq来匹配发送和接受
    '''

    def __init__(self, write_callback, response_callback):
        self._session_id = 0
        self._frame = srp_frame.Frame(self._onNewMessageCallback)
        self._write = write_callback
        self._response_callback = response_callback
        self._system_state_callback = None
        self._hardware_state_callback = None
        self._laser_point_callback = None
        self._notify_move_task_finished_callback = None
        self._notify_action_task_finished_callback = None
        self._notify_mission_list_change_callback = None

    def set_system_state_callback(self, fun):
        self._system_state_callback = fun

    def set_hardware_state_callback(self, fun):
        self._hardware_state_callback = fun

    def set_laser_point_callback(self, fun):
        self._laser_point_callback = fun

    def set_notify_move_task_finished_callback(self, fun):
        self._notify_move_task_finished_callback = fun

    def set_notify_action_task_finished_callback(self, fun):
        self._notify_action_task_finished_callback = fun

    def set_notify_mission_list_change_callback(self, fun):
        self._notify_mission_list_change_callback = fun

    def onRead(self, data):
        self._frame.setNewFrame(data)

    def login(self, seq, user_name, passwd, session_id=0):
        md5 = hashlib.md5()
        md5.update(passwd.encode('utf8'))
        self._session_id = session_id

        login_request = main_pb2.LoginRequest(username=user_name, password=md5.hexdigest())
        self._sendRequestMsg(seq, main_pb2.Request.REQUEST_LOGIN, login_request=login_request)

    def request_all_state(self, seq):
        self._sendRequestMsg(seq, main_pb2.Request.REQUEST_ALL_STATE)

    def request_system_state(self, seq):
        self._sendRequestMsg(seq, main_pb2.Request.REQUEST_SYSTEM_STATE)

    def request_hardware_state(self, seq):
        self._sendRequestMsg(seq, main_pb2.Request.REQUEST_HARDWARE_STATE)

    def setEmergency(self, seq):
        self._send_command_msg(seq, main_pb2.CMD_TRIGGER_EMERGENCY)

    def set_user_set_state(self, seq):
        self._send_command_msg(seq, main_pb2.CMD_SET_HMI_STATE, paramInt=4, paramInt1=3000)

    def set_auto_upload_laser_point(self, seq, enable):
        if enable:
            self._send_command_msg(seq, main_pb2.CMD_ENABLE_AUTO_UPLOAD_LASER_POINT)
        else:
            self._send_command_msg(seq, main_pb2.CMD_DISABLE_AUTO_UPLOAD_LASER_POINT)

    def cancelEmergencyState(self, seq):
        self._send_command_msg(seq, main_pb2.CMD_CANCEL_EMERGENCY)

    def map_switching(self, seq, map_name, locate_station_id, locate_pose=Pose(0, 0, 0), is_force_locate=False):
        self._send_command_msg(seq, main_pb2.CMD_MAP_SWITCHING, paramStr=map_name, paramInt=locate_station_id,
                               pose=locate_pose, param_boolean=is_force_locate)

    def move_to_station(self, seq, no, station_id, avoid_policy=main_pb2.MovementTask.OBSTACLE_AVOID_WAIT, station_ids=[]):
        movement_task = main_pb2.MovementTask(
            no=no,
            type=main_pb2.MovementTask.MT_MOVE_TO_STATION,
            stations=[station_id] if len(station_ids) == 0 else station_ids,
            avoid_policy=avoid_policy
        )

        self._send_command_msg(seq, main_pb2.CMD_NEW_MOVEMENT_TASK, movement_task=movement_task)

    def move_follow_path(self, seq, no, paths, avoid_policy=main_pb2.MovementTask.OBSTACLE_AVOID_WAIT, checkpint_no=0):
        movement_task = main_pb2.MovementTask(
            no=no,
            type=main_pb2.MovementTask.MT_MOVE_FOLLOW_PATH,
            avoid_policy=avoid_policy
        )
        for path in paths:
            proto_path = movement_task.paths.add()
            proto_path.type = path.type.value
            proto_path.sx = path.sx
            proto_path.sy = path.sy
            proto_path.ex = path.ex
            proto_path.ey = path.ey
            proto_path.cx = path.cx
            proto_path.cy = path.cy
            proto_path.dx = path.dx
            proto_path.dy = path.dy
            proto_path.radius = path.radius
            proto_path.rotate_angle = path.rotate_angle
            proto_path.direction = path.direction.value
            proto_path.limit_v = path.limit_v
            proto_path.limit_w = path.limit_w

        self._send_command_msg(seq, main_pb2.CMD_NEW_MOVEMENT_TASK, movement_task=movement_task, paramInt=checkpint_no)

    def set_checkpoint(self, seq, checkpoint_no):
        self._send_command_msg(seq, main_pb2.CMD_SET_CHECKPOINT, paramInt=checkpoint_no)


    def pause_movement(self, seq, pause_level):
        self._send_command_msg(seq, main_pb2.CMD_PAUSE_MOVEMENT, paramInt=pause_level)

    def continue_movement(self, seq):
        self._send_command_msg(seq, main_pb2.CMD_CONTINUE_MOVEMENT)

    def set_current_map(self, seq, map_name):
        self._send_command_msg(seq, main_pb2.CMD_SET_CUR_MAP, paramStr=map_name)


    def cancel_movement_task(self, seq):
        self._send_command_msg(seq, main_pb2.CMD_CANCEL_MOVEMENT_TASK)

    def excute_action_task(self, seq, no, action_id, param0, param1, param2 = 0):
        action_task = main_pb2.ActionTask(
            no=no,
            id=action_id,
            param0=param0,
            param1=param1,
            param2=param2
        )
        self._send_command_msg(seq, main_pb2.CMD_NEW_ACTION_TASK, action_task=action_task)

    def cancel_action_task(self, seq):
        self._send_command_msg(seq, main_pb2.CMD_CANCEL_ACTION_TASK)

    def readInputRegisters(self, seq, start_addr, count):
        self._sendRequestMsg(seq, main_pb2.Request.REQUEST_READ_INPUT_REGISTER, param_int=start_addr, param_int1=count)

    def get_config(self, seq):
        self._sendRequestMsg(seq, main_pb2.Request.REQUEST_LOAD_CONFIG)

    def set_config(self, seq, configs):
        '''
        设置参数
        :param seq:
        :param configs: dict （key - value）
        :return:
        '''
        items = []
        for key, value in configs.items():
            print(key, value)
            item = main_pb2.ConfigItem(key=key, value=str(value))
            items.append(item)

        self._sendRequestMsg(seq, main_pb2.Request.REQUEST_SAVE_CONFIG, config=items)

    def _send_command_msg(self, seq, cmdType, paramInt=None, paramInt1=None, paramStr=None, pose=None,
                          param_boolean=False,
                          movement_task=None,
                          action_task=None, missionLst=None, lockerIP=None, lockerNickname=None):
        command = main_pb2.Command(
            command=cmdType,
            param_int=paramInt,
            param_int1=paramInt1,
            param_boolean=param_boolean,
            movement_task=movement_task,
            action_task=action_task,
            param_str=paramStr,
            mission_list=missionLst,
            locker_ip_address=lockerIP,
            locker_nickname=lockerNickname
        )
        if pose is not None:
            command.pose = main_pb2.Pose(x=pose.x, y=pose.y, yaw=pose.yaw)

        message = main_pb2.Message(
            type=main_pb2.MSG_COMMAND,
            seq=seq,
            session_id=self._session_id,
            command=command)

        self._sendMsg(message)

    def _sendRequestMsg(self, seq, request_type, login_request=None, file_op=None, param_str=None, param_str1=None,
                        config=None, param_int=None, param_int1=None):
        request = main_pb2.Request(
            request_type=request_type,
            login_request=login_request,
            file_op=file_op,
            param_str=param_str,
            param_str1=param_str1,
            param_int=param_int,
            param_int1=param_int1,
            config=config)

        message = main_pb2.Message(
            type=main_pb2.MSG_REQUEST,
            seq=seq,
            session_id=self._session_id,
            request=request)

        self._sendMsg(message)

    def _send_response_msg(self, response_type, seq, notify_type):
        notification_response = main_pb2.NotifyResponse(
            ack=True,
            type=notify_type
        )

        response = main_pb2.Response(
            response_type=response_type,
            notify_response=notification_response
        )

        message = main_pb2.Message(
            type=main_pb2.MSG_RESPONSE,
            seq=seq,
            session_id=self._session_id,
            response=response)

        self._sendMsg(message)

    def _sendMsg(self, message):
        msg = message.SerializeToString()
        frame = srp_frame.Frame.buildFrame(msg)
        print('\n')
        print(frame.hex())
        print('\n')

        self._write(frame)

    def _handleRecvResponseMsg(self, msg):
        response = msg.response
        # logging.debug(response)
        value = {}
        # 不需要回复ResponseResult
        if response.response_type == main_pb2.Response.RESPONSE_COMMON_POSE_INFO:
            _logger.debug(len(response.common_poses_info.car_simulate_poses))
        elif response.response_type == main_pb2.Response.RESPONSE_LASER_POINTS:
            # _logger.debug(len(response.laser_points.xs))
            # _logger.debug(len(response.laser_points.xs1))
            pass
        else:  # 需要回复ResponseResult
            if response.result.result_state == main_pb2.ResponseResult.RESPONSE_OK or response.result.result_state == main_pb2.ResponseResult.RESPONSE_PROCESSING:
                if response.response_type == main_pb2.Response.RESPONSE_LOGIN:
                    self._session_id = msg.session_id
                elif response.response_type == main_pb2.Response.RESPONSE_ALL_STATE:
                    value = {"system_state": MessageToDict(response.system_state),
                             "hardware_state": MessageToDict(response.hardware_state)}
                elif response.response_type == main_pb2.Response.RESPONSE_SYSTEM_STATE:
                    if self._system_state_callback is not None:
                        self._system_state_callback(response.system_state)
                elif response.response_type == main_pb2.Response.RESPONSE_HARDWARE_STATE:
                    if self._hardware_state_callback is not None:
                        self._hardware_state_callback(response.hardware_state)
                    pass
                elif response.response_type == main_pb2.Response.RESPONSE_READ_INPUT_REGISTER:
                    value = response.registers
                elif response.response_type == main_pb2.Response.RESPONSE_LOAD_CONFIG:
                    dict_obj = MessageToDict(response)
                    configs = dict_obj['config']
                    # 剔除空的选项
                    i = 0
                    while i < len(configs):
                        if not 'value' in configs[i]:
                            configs.pop(i)
                            continue
                        i += 1
                    value = configs

                self._response_callback(msg.seq, response.response_type, True, value=value)
            else:
                self._response_callback(msg.seq, response.response_type, False, result_code=response.result.result_code)

    def _ack_notification(self, message):
        if message.seq <= 0:
            return
        self._send_response_msg(main_pb2.Response.RESPONSE_NOTIFY, message.seq, message.notification.notify_type)

    def _handle_recv_notification_msg(self, notification):
        if notification.notify_type == main_pb2.Notification.NOTIFY_MOVE_TASK_FINISHED:
            if self._notify_move_task_finished_callback is not None:
                self._notify_move_task_finished_callback(notification.movement_task)
        elif notification.notify_type == main_pb2.Notification.NOTIFY_ACTION_TASK_FINISHED:
            if self._notify_action_task_finished_callback is not None:
                self._notify_action_task_finished_callback(notification.action_task)
        elif notification.notify_type == main_pb2.Notification.NOTIFY_MISSION_LIST_CHANGED:
            if self._notify_mission_list_change_callback is not None:
                self._notify_mission_list_change_callback(notification.mission_list)
        else:
            raise RuntimeError('UNRACHABLE! notify type: ', notification.notify_type)

    def _onNewMessageCallback(self, msg):
        try:
            message = main_pb2.Message()
            message.ParseFromString(msg)
        except BaseException as e:
            print(e)
            raise e

        if message.type == main_pb2.MSG_RESPONSE:
            self._handleRecvResponseMsg(message)
        elif message.type == main_pb2.MSG_NOTIFICATION:
            self._ack_notification(message)
            self._handle_recv_notification_msg(message.notification)
            print(message)
        else:
            raise Exception('UNREACHABLE! ' + str(message.type))


if __name__ == '__main__':
    message = main_pb2.Message()
    try:
        message.ParseFromString(bytes.fromhex(
            '00000178080119835486b9780100002aea0208021ae10210021803221c10c897feffffffffffff0118938a0238cbe7ffffffffffffff0140642801320208023a9301100518022a01455001621a080410a2a1feffffffffffff0118828a0258c51868ffffffff076226080110a2a1feffffffffffff0118828a0220c097feffffffffffff0128828a0250a08d0668646214080410c097feffffffffffff0118828a0258ce1872260a0e3030313930304448303031393436189ae193ad03200428ceffffffffffffffff0130be18900198a1ccd38b2f4211100518022007280138015098a1ccd38b2f4845520e44473541545332303230353030366001680178018001018801149201009a012f0a0e3030313930304448303031393436189ae193ad0320fbffffffffffffffff0128cfffffffffffffffff0130c518a00164a80102b20100c80101d001b4911ae80101fa01260a0e3030313930304448303031393436189ae193ad03200428ceffffffffffffffff0130be1852021002')[4:])
    except BaseException as e:
        print(e)
    print(message)

    '''
    000000321501000000222b12290a0561646d696e12203231323332663239376135376135613734333839346130653461383031666333
    注意前面四个字节是头（即协议长度），解析出来的数据是：
    seq: 1
    request {
      login_request {
        username: "admin"
        password: "21232f297a57a5a743894a0e4a801fc3"
      }
    }
    827e00840000008008011979dc77ca780100002a7308021a6b1002180122003202080252064e4f5f4d41506001680278018001018801649201009a0100a0013ca80102b20100c80102da0109cbec38ebb52b8b8e19e2010c08cbec38105220aabcd58306e2010a08ebb52b20afbcd58306e2010a088b8e1920afbcd58306e80101fa010052021002
    这种情况要去掉前面8个字节，估计最前面四个字节是websocket的封装，具体没有去研究
    '''
