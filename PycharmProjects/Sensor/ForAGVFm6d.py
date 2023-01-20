import os
import numpy as np
import matplotlib.pyplot as plt


path = "test(1).txt"
if not os.path.isfile(path):
    print("file does not exist!")

f = open(path, "r")
line_datas = f.readlines()
# print(line_datas)
pose = []  # x y z roll pitch yaw

for i in range(len(line_datas)):
    tr = []
    line_data = line_datas[i].split(" ")

    for data in line_data:
        if len(data.split(":")) == 2:
            tr.append(float(data.split(":")[1]))
    pose.append(np.array(tr))

pose = np.array(pose).reshape(-1, len(tr))
# print(pose)
# print(pose[:, :3])

pose[:, :3] = pose[:, :3]
pose = pose - np.mean(pose, 0)  # 去均值，但不归一
# print(pose)

name = ['x', 'y', 'theta']
x_name = ['mm', 'mm', 'mm']
y_name = ['num', 'num', 'num']
my_x_ticks = np.arange(0, 13, 1)#原始数据有13个点，故此处为设置从0开始，间隔为1
# print(my_x_ticks)
print(pose.shape[1])
group_num = 100  # 直方图组数
for i in range(pose.shape[1]):
    data = pose[:, i]
    plt.figure()
    plt.title(name[i]+" max-min:{:.4f} {}".format(max(data)-min(data), x_name[i]))
    plt.xlabel(x_name[i])
    # print(x_name[1])
    plt.ylabel(y_name[i])
    plt.hist(data, np.linspace(min(data), max(data), group_num), histtype='bar', rwidth=0.8)
    # plt.bar(np.arange(min(data),max(data),1),heigth=len(data))
    # plt.xticks(np.arange(min(data),max(data),1))
    plt.show()
