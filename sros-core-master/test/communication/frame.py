#!/usr/bin/env python
# -*- coding:utf-8 -*-
# file frame.py
# author pengjiali
# date 19-6-24.
# copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# describe 收到的数据只管往本类中堆，本类中处理粘包、断包的问题, 注释参见Frame.js


class Frame:
    '''本类处理protobuf协议拼包和切包'''

    def __init__(self, on_new_frame_callback):
        self._on_new_frame_callback = on_new_frame_callback
        self._buffer = bytearray()
        self._frame = bytearray()
        self._validLength = 0

    def clear(self):
        self._validLength = 0

    def setNewFrame(self, frame):
        if self._validLength == 0:
            start_index = self._loopSliceAFrame(frame, 0, len(frame))
            if start_index == len(frame):
                return
            else:
                remanent_data = frame[start_index: len(frame)]
                self._frame = bytearray(remanent_data)
                self._validLength += len(remanent_data)
                return
        else:
            self._frame[self._validLength:] = frame
            self._validLength += len(frame)

        start_index = self._loopSliceAFrame(self._frame, 0, self._validLength)
        self._frame[0: start_index] = []
        self._validLength -= start_index

    def _loopSliceAFrame(self, buffer, start_index, valid_len):
        while valid_len > 4:
            protobuf_len = 0
            tmp = buffer[start_index]
            tmp <<= 24
            protobuf_len += tmp
            tmp = buffer[start_index + 1]
            tmp <<= 16
            protobuf_len += tmp
            tmp = buffer[start_index + 2]
            tmp <<= 8
            protobuf_len += tmp
            tmp = buffer[start_index + 3]
            tmp <<= 0
            protobuf_len += tmp

            total_len = protobuf_len + 4

            if total_len <= valid_len:
                self._on_new_frame_callback(buffer[start_index + 4: start_index + total_len])
                start_index += total_len
                valid_len -= total_len
            else:
                break

        return start_index

    @staticmethod
    def buildFrame(msg):
        frame = bytearray()
        msg_len = len(msg)
        frame.insert(0, msg_len >> 24 & 0xFF)
        frame.insert(1, msg_len >> 16 & 0xFF)
        frame.insert(2, msg_len >> 8 & 0xFF)
        frame.insert(3, msg_len >> 0 & 0xFF)
        frame[4:] = msg
        return frame
