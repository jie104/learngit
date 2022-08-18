#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @File: udp_server_test.py
# @Author: pengjiali
# @Date: 19-11-4.
# @Copyright: Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# @Describe:

import unittest
import socketserver
import threading
import time


class UDPServerTest(unittest.TestCase):
    '''
    测试udp_server.py这个好不好使
    '''

    def __init__(self, *args, **kwargs):
        unittest.TestCase.__init__(self, *args, **kwargs)

    @classmethod
    def setUpClass(self):
        '''
        TODO(pengjiali): 通过配置参数设置如下参数：
        1. 将network.server_ip 改为本机ip
        2. 将network.server_port 设置为： 8001
        3. 将network.enable_udp_upload_ip_info 设置为：True
        4. 将network.ip_info_upload_freq 设置为:3
        :return:
        '''
        pass

    @classmethod
    def tearDownClass(self):
        pass

    def setUp(self):
        # 每个测试用例执行之前做操作
        pass

    def tearDown(self):
        # 每个测试用例执行之后做操作
        pass

    def test_ip_upload(self):
        '''
        测试agv是否正常上传ip，上传ip间隔时间
        :return:
        '''
        result = {
            "is_receive_data": False,
            "first_receive_time": None,
            "second_receive_time": None,
        }

        class UDPHandler(socketserver.BaseRequestHandler):
            def handle(self):
                data = self.request[0].strip()
                socket = self.request[1]
                print("{} wrote:".format(self.client_address[0]))
                print(data)

                if not self.server.result["is_receive_data"]:
                    self.server.result["is_receive_data"] = True
                    self.server.result["first_receive_time"] = time.time()
                else:
                    self.server.result["second_receive_time"] = time.time()

        HOST, PORT = "192.168.83.189", 8001

        with socketserver.UDPServer((HOST, PORT), UDPHandler) as server:
            server.result = result

            def stop_loop(server):
                time.sleep(7)
                server.shutdown()

            t = threading.Thread(target=stop_loop, args=(server,))
            t.start()

            server.serve_forever()

        print(result)
        self.assertTrue(result["is_receive_data"])
        self.assertIsNotNone(result["second_receive_time"])
        self.assertLessEqual(result["second_receive_time"] - result["first_receive_time"], 3 + 1)


if __name__ == '__main__':
    unittest.main()  # 运行所有的测试用例
