# _*_ coding: utf-8 _*_
"""
Time:   2022/3/11 3:10
Author: zxj
Version: V 2.0
File: normality_histogram_v3.py
Describe: 读取站点的所有数据并进行统计
"""


from scipy.stats import norm
import re
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams['font.sans-serif'] = ['SimHei'] # 步骤一（替换sans-serif字体）
plt.rcParams['axes.unicode_minus'] = False  # 步骤二（解决坐标轴负数的负号显示问题）


###############################################################
#为满足测试需求，针对文件格式，需要修改的地方可能有五处，如下
###############################################################

#1、读取的文件名
list=['1.txt']

#2、二维码ID
key1 = '001900DH000000'

#3、到一个站点的数据重复读取的次数
rep=2

#4、数据每隔N行重复出现
N=300

#5、pose_x,pose_y所在行位置
i_x=-4
i_y=-3

#初始参数
l1=[]
l2=[]
L1=L2=[]
flag=False
k=0
l=0
sum_x=sum_y=0
k2=0

for x in list:
    f = open(x,"r",encoding="utf-8")
    for line in f.readlines():
        line = re.split(',', line)
        if key1 in line:
            l1.append(float(line[i_x]))
            l2.append(float(line[i_y]))
            k=0
            k2+=1

        else:
            k+=1
            if k%N==0:
                print("第%d次读不到数据，所对应的行时间：%s"%(l+1,line[0]))
                k=0
                l+=1
                k2+=1

n=int(len(l1)/rep)
L1=[0]*n
L2=[0]*n

for i in range(n):
    L1[i]=l1[rep*i]
    L2[i]=l2[rep*i]

sum_x=sum(L1)
sum_y=sum(L2)

print("循环次数：%d"%(k2/rep))

# pose平均值
mean_x = sum_x / len(L1)
mean_y = sum_y / len(L2)


l_x=[]
l_y=[]
for j in range(n):
    l_x.append((L1[j]-mean_x)*1000)
    l_y.append((L2[j]-mean_y)*1000)


fig=plt.figure()

#直方图
group_num = 50  # 直方图组数

#绘制图形1----------------------------------------------------------------------------------------------------------------------------------
ax1=fig.add_subplot(211)
mu = np.mean(l_x)*1000  # 计算均值
range=max(l_x)-min(l_x)   #计算极差
sigma = np.std(l_x)  # 计算标准差

tumor_min_1=min(l_x)
tumor_max_1=max(l_x)
n, bins, patches = plt.hist(l_x, group_num, density=True, edgecolor="black", facecolor='gray', alpha=0.6,
                            range=[int(tumor_min_1), int(tumor_max_1)])
y = norm.pdf(bins, mu, sigma)  # 拟合一条最佳正态分布曲线y
plt.plot(bins, y, 'g--')  # 绘制y的曲线

ax1.set_title('X range=%.2f mm,mean=%.2f mm,std=%.2f mm,3σ=%.2f mm'%(range,-mu,sigma,3*sigma))
ax1.set_xlabel('locate_error(mm)')
ax1.set_ylabel("possibility")

#绘制图形2------------------------------------------------------------------------------------------------------------------------------------
ax2=fig.add_subplot(212)

mu1 = np.mean(l_y)*1000  # 计算均值
range1=max(l_y)-min(l_y)
sigma1 = np.std(l_y)  # 计算标准差

tumor_min_2=min(l_y)
tumor_max_2=max(l_y)
n1, bins1, patches1 = plt.hist(l_y, group_num, density=True, edgecolor="black", facecolor='gray', alpha=0.6,
                            range=[int(tumor_min_2), int(tumor_max_2)])
y1 = norm.pdf(bins1, mu1, sigma1)  # 拟合一条最佳正态分布曲线y
plt.plot(bins1, y1, 'g--')  # 绘制y的曲线

ax2.set_title('Y range=%.2f mm,mean=%.2f mm,std=%.2f mm,3σ=%.2f mm'%(range1,-mu1,sigma1,3*sigma1))
ax2.set_xlabel('location_error(mm)')
ax2.set_ylabel("possibility")

#===========================================================================================================================================================
plt.suptitle('location error distribution,can not scan %d '%l,size=19)
plt.show()

