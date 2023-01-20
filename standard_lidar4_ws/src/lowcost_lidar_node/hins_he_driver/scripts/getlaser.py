#!/usr/bin/env python
# -*- coding: UTF-8 -*-  
import rospy
from sensor_msgs.msg import LaserScan
import os
from datetime import datetime

global save_dir, file_path, data_index_infile, data_index_max, file_index, filen_base_name

save_dir = os.getcwd()+"/rich_data"                # 文件保存路径
file_path = ""                                                     # 文件全路径
data_index_infile = 1                                     # 文件内雷达数据数量
data_index_max = 100                                  # 文件内雷达数据的最大值
file_index = 0                                                     # 文件名的后缀
filen_base_name = "rich_data"              # 文件的基础名

# 检查路径是否存在，不存在则递归创建
def CheckDir(dir):
    if(not os.path.exists(dir)):
        os.makedirs(dir)        # 递归创建文件夹

# 检查文件名是否存在，存在则增加文件序号
def CheckFileExist():
    global file_path, filen_base_name, file_index, save_dir
    file_name = filen_base_name+"_"+str(file_index)+".txt"      # 文件全名
    file_path = save_dir+"/"+file_name  
    while(os.path.exists(file_path)):   # 如果有重复名字，则文件序号++
        file_index += 1
        file_name = filen_base_name+"_"+str(file_index)+".txt"
        file_path = save_dir+"/"+file_name

# 将雷达数据写入file_path文件中
def WriteData(file_path, data_index_infile, data):
    dt = datetime.now()
    time_stamp = dt.isoformat(sep=' ')
    with open(file_path,"a+") as f:            # a+:可读、可写，文件不存在先创建，不会覆盖，追加在末尾
        f.write(time_stamp + "\r")                 # 加入时间戳
        f.write("=========="+str(data_index_infile)+"=========="+"\r")              
        for r in data.ranges:   
            f.write(str(r)+" ")                                 # 写入数据
        f.write("\r")
        f.write("\r")              
    data_index_infile += 1
    print("filepath:"+file_path+"    data num:"+str(data_index_infile)+" record finish")

def LaserCallback(data):
    global file_path, data_index_infile, data_index_max, filen_base_name, file_index, save_dir
    if(data_index_infile>data_index_max):                   # 文件内数量超过最大值，则新建文件
        data_index_infile = 1
        file_index += 1
    file_name = filen_base_name+"_"+str(file_index)+".txt"      # 文件全名
    file_path = save_dir+"/"+file_name  
    WriteData(file_path, data_index_infile, data)



def laser_listener():
    rospy.init_node('laser_listener', anonymous=True)
    rospy.Subscriber("scan", LaserScan,LaserCallback,queue_size = 1)
    rospy.spin()

if __name__ == '__main__':
    CheckDir(save_dir)                             # 检查保存数据的文件夹是否存在，不存在则递归创建文件夹
    CheckFileExist()                                    # 检查文件序号是否已经存在，已存在则改变文件序号
    laser_listener()
