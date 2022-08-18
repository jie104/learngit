#!/usr/bin/env python
# -*- coding:utf-8 -*-
# file path.py
# author pengjiali
# date 19-6-29.
# copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# describe

from enum import Enum


class Direction(Enum):
    Forward = 1
    BackForward = 2


class PathType(Enum):
    PATH_ZERO = 0
    PATH_LINE = 1
    PATH_BEZIER = 3
    PATH_ROTATE = 4


class Path:
    def __init__(self):
        # 距离单位均为mm
        self.type = PathType.PATH_ZERO

        self.sx = 0  # 起点
        self.sy = 0

        self.cx = 0
        self.cy = 0

        self.dx = 0
        self.dy = 0

        self.ex = 0  # 终点
        self.ey = 0

        self.rotate_angle = 0  # 角度单位为1/1000 rad
        self.radius = 0
        self.direction = Direction.Forward  # 0x01->前进；0x02->后退

        self.limit_v = 0  # 速度限制
        self.limit_w = 0

    @staticmethod
    def create_line_path(sx, sy, ex, ey, direction=Direction.Forward):
        path = Path()
        path.type = PathType.PATH_LINE
        path.sx = sx
        path.sy = sy
        path.ex = ex
        path.ey = ey
        path.direction = direction
        return path

    @staticmethod
    def create_bezier_path(sx, sy, cx, cy, dx, dy, ex, ey, direction=Direction.Forward):
        path = Path()
        path.type = PathType.PATH_BEZIER
        path.sx = sx
        path.sy = sy
        path.ex = ex
        path.ey = ey
        path.cx = cx
        path.cy = cy
        path.dx = dx
        path.dy = dy
        path.direction = direction
        return path

    @staticmethod
    def create_rotate_path(rotate_angle):
        path = Path()
        path.type = PathType.PATH_ROTATE
        path.rotate_angle = rotate_angle
        return path