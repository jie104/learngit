#!/usr/bin/env  python
#coding:utf-8
import rospy
from hins_he_driver.srv import *
import sys
"""
    需求: 
        编写两个节点实现服务通信，客户端节点需要提交两个整数到服务器
        服务器需要解析客户端提交的数据，相加后，将结果响应回客户端，
        客户端再解析

    客户端实现:
        1.导包
        2.初始化 ROS 节点
        3.创建请求对象
        4.发送请求
        5.接收并处理响应

    优化:
        加入数据的动态获取


"""
if __name__ == "__main__":
    # 2.初始化 ROS 节点
    rospy.init_node("Hins_debug")
    # 3.创建请求对象
    client = rospy.ServiceProxy("hins_he_area_data",hins_srv)
    # 请求前，等待服务已经就绪
    # 方式1:
    # rospy.wait_for_service("AddInts")
    # 方式2
    client.wait_for_service()
    # 4.发送请求,接收并处理响应
    # 方式1
    # resp = client(3,4)
    # 方式2
    # resp = client(AddIntsRequest(1,5))
    # 方式3
    #req = AddintsRequest()
    # req.num1 = 100
    # req.num2 = 200 
    while(1):
        resp = client(int(1))
    #优化
    #req.num1 = int(sys.argv[1])
    #req.num2 = int(sys.argv[2]) 

    #resp = client.call(req)
        rospy.loginfo("1通道:%d, 2通道:%d, 3通道:%d",resp.area1, resp.area2, resp.area3)


