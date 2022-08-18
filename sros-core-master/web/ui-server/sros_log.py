#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @File: sros_log.py.py
# @Author: pengjiali
# @Date: 20-3-28
# @Copyright: Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# @Describe:


import logging
import logging.handlers
import os


class SrosLog():
    '''
    SROS中python进程的日志管理类，所有进程python进程都必须引用此类来生成日志。
    使用方法：
    1.在所有进程开始的地方初始化该类，并调用sendLogToFile()函数
    2.若需要输出到控制台的，手动调用一下sendLogToConsole()函数
    3.所有需要输出日志的类调用如下代码：
        import logging
        _logger = logging.getLogger(__name__)
    4.在需要输出日志的地方插入日志输出，如下代码：
        _logger.info("info: sros log example")
        _logger.error("error： sros log example")
    '''

    def __init__(self, module_name):
        '''
        初始化sros日志
        :param module_name: 日志的模块名
        '''
        self._format = ('%(asctime)-15s %(levelno)-1d    %(threadName)-1s %(filename)-1s:%(lineno)-s] %(message)s')
        self._module_name = module_name  # 模块名，日志输出用这个名字

        self._log = logging.getLogger()
        self._log.setLevel(logging.NOTSET)

    def sendLogToConsole(self, level=logging.DEBUG):
        '''
        将日志打印到控制台，默认不打印
        :param level:
        :return:
        '''
        handler = logging.StreamHandler()
        handler.setLevel(level)
        handler.setFormatter(logging.Formatter(self._format))
        self._log.addHandler(handler)

    def sendLogToFile(self, level=logging.INFO):
        '''
        所有的进程日志都会放到/sros/log/目录下，除了主进程sros外，其他的进程都用进程名的文件夹包裹。
        参见：http://wiki.standard-robots.com/wiki/doku.php?id=project:sros:process_manager
        :param level:
        :return:
        '''

        sros_log_dir = "/sros/log/"
        source_log_dir = sros_log_dir + self._module_name + "/"
        if not os.path.exists(source_log_dir):
            os.makedirs(source_log_dir)

        handler = logging.handlers.TimedRotatingFileHandler(source_log_dir + self._module_name + ".log",
                                                            when='D', interval=1, backupCount=10)
        handler.setLevel(level)
        handler.setFormatter(logging.Formatter(self._format))
        self._log.addHandler(handler)


import logging

sros_log = SrosLog("web_code")
sros_log.sendLogToFile(logging.INFO)
_logger = logging.getLogger()
