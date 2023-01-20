import sys,re,csv
import matplotlib.pyplot as plt


plt.rcParams['font.sans-serif'] = ['SimHei'] # 步骤一（替换sans-serif字体）
plt.rcParams['axes.unicode_minus'] = False  # 步骤二（解决坐标轴负数的负号显示问题）

path = "C:\Users\张赟\Desktop\测试\畸形运动测试\my_test\需要求最优的数据.tar(1) - 副本\需要求最优的数据" #文件夹目录
files= os.listdir(path) #得到文件夹下的所有文件名称
list=['-140~ -134.txt']
for x in list:
    l1,l2,l3,l4,l5,l6=[],[],[],[],[],[]
    f = open(x,"r",encoding="utf-8")
    for line in f.readlines():
        line=line.split()
        l1.append(float(line[-3]))
        l2.append(float(line[-2]))
        l3.append(float(line[-1]))
        l4.append(float(line[0]))
        l5.append(float(line[1]))
        l6.append(float(line[2]))

    print("//-------------------------------------------------------",x)


    #标定后的雷达机械参数
    complete_range_x=max(l1)-min(l1)
    complete_range_y=max(l2)-min(l2)
    complete_range_θ=max(l3)-min(l3)
    complete_mean_x=sum(l1)/len(l1)
    complete_mean_y=sum(l2)/len(l2)
    complete_mean_θ=sum(l3)/len(l3)

    print('标定好的x值极差:',complete_range_x,'mm')
    print('标定好的y值极差：',complete_range_y,'mm')
    print('标定好的θ值极差：',complete_range_θ,'°')
    print('标定好的x值平均值:',complete_mean_x,'mm','   标定好的y值平均值：',complete_mean_y,'mm','   标定好的θ值平均值：',complete_mean_θ,'°')

