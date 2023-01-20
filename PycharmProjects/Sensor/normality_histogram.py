from scipy.stats import norm
import sys,re,csv
import matplotlib.pyplot as plt
import numpy as np
from math import sqrt


plt.rcParams['font.sans-serif'] = ['SimHei'] # 步骤一（替换sans-serif字体）
plt.rcParams['axes.unicode_minus'] = False  # 步骤二（解决坐标轴负数的负号显示问题）
# list=['sros22.txt','sros23.txt','sros24.txt','sros25.txt','sros26.txt','sros27.txt','sros28.txt','sros29.txt','sros30.txt',
#       'sros31.txt','sros32.txt','sros33.txt','sros34.txt']



list=['pepperfuch.txt']
l1=[]
l2=[]
l3=[]
L1=[]
L2=[]

key1 = '001900DH002073'
key2='001900DH002077'
flag=False
k=0
l=0
sum_x=sum_y=0
k2=0

for x in list:
    f = open(x,"r",encoding="utf-8")
    for line in f.readlines():
            # print(line)
        line = re.split(',', line)
        if key1 in line:
            l1.append(float(line[-3]))
            l2.append(float(line[-2]))
            # if float(line[2]) >0.235:
            #     print(line)
            k=0
            sum_x+=float(line[-3])
            sum_y+=float(line[-2])
            L1.append(float(line[-3]))
            L2.append(float(line[-2]))
            k2+=1

        else:
            k+=1
            if k>6:
                if k%6==0:
                    print(float(line[-3]))
                    l1.append(0.03)
                    l2.append(0.03)
                    l3.append(float(line[-1]))
                    k=0
                    l+=1

print(k2)
# pose平均值
mean_x = sum_x / len(l1)
mean_y = sum_y / len(l2)




for i in range(len(l1)):
    l1[i]=(l1[i]-mean_x)*1000
    l2[i]=(l2[i]-mean_y)*1000

# Data=[]
# Data.append(l1)
# Data.append(l2)
#
# for data in Data:
#     mu = np.mean(data)*1000  # 计算均值
#     range=max(data)-min(data)
#     sigma = np.std(data)  # 计算标准差
#     print(data)
#     num_bins = 20  # 直方图柱子的数量
#
#     tumor_min=min(data)
#     tumor_max=max(data)
#     print(mu)
#     print(sigma)
#     n, bins, patches = plt.hist(data, num_bins, density=True, edgecolor="black", facecolor='gray', alpha=0.6,
#                             range=[int(tumor_min), int(tumor_max)])
#     # for i in range(len(n)):
#     #     n[i]=n[i]*100
#     # 直方图函数，x为x轴的值，normed=1表示为概率密度，即和为一，绿色方块，色深参数0.5.返回n个概率，直方块左边线的x值，及各个方块对象
#     # print(n)
#     # print(bins)
#     # print(patches)
#     y = norm.pdf(bins, mu, sigma)  # 拟合一条最佳正态分布曲线y
#     plt.plot(bins, y, 'g--')  # 绘制y的曲线
#
#     plt.title("X方向定位精度"+'\n'
#               +'极差=%.2f mm,均值=%.2f mm,标准差=%.2f mm,3σ=%2.f mm'%(range,mu,sigma,3*sigma))
#     plt.xlabel('定位误差(mm)')  # 绘制x轴
#     plt.ylabel('统计概率')  # 绘制y轴
#     plt.subplots_adjust(left=0.15)  # 左边距
#     plt.show()


#
# #繪製圖形
fig=plt.figure()
# plt.title("10.31 15:13-11.3 13:46 昆山18小车pose")

ax1=fig.add_subplot(211)



#直方图
group_num = 25  # 直方图组数

#绘制图形1----------------------------------------------------------------------------------------------------------------------------------
mu = np.mean(l1)*1000  # 计算均值
range=max(l1)-min(l1)
sigma = np.std(l1)  # 计算标准差
print(l1)

tumor_min_1=min(l1)
tumor_max_1=max(l1)
n, bins, patches = plt.hist(l1, group_num, density=True, edgecolor="black", facecolor='gray', alpha=0.6,
                            range=[int(tumor_min_1), int(tumor_max_1)])
    # for i in range(len(n)):
    #     n[i]=n[i]*100
    # 直方图函数，x为x轴的值，normed=1表示为概率密度，即和为一，绿色方块，色深参数0.5.返回n个概率，直方块左边线的x值，及各个方块对象
    # print(n)
    # print(bins)
    # print(patches)
y = norm.pdf(bins, mu, sigma)  # 拟合一条最佳正态分布曲线y
plt.plot(bins, y, 'g--')  # 绘制y的曲线


ax1.set_title('X方向定位精度 极差=%.2f mm,均值=%.2f mm,标准差=%.2f mm,3σ=%.2f mm'%(range,mu,sigma,3*sigma))
ax1.set_xlabel('定位误差(mm)')

ax1.set_ylabel("统计概率")

#绘制图形2------------------------------------------------------------------------------------------------------------------------------------
ax2=fig.add_subplot(212)

mu1 = np.mean(l2)*1000  # 计算均值
range1=max(l2)-min(l2)
sigma1 = np.std(l2)  # 计算标准差
print(l2)

tumor_min_2=min(l2)
tumor_max_2=max(l2)
n1, bins1, patches1 = plt.hist(l2, group_num, density=True, edgecolor="black", facecolor='gray', alpha=0.6,
                            range=[int(tumor_min_2), int(tumor_max_2)])
print(n1)
    # for i in range(len(n)):
    #     n[i]=n[i]*100
    # 直方图函数，x为x轴的值，normed=1表示为概率密度，即和为一，绿色方块，色深参数0.5.返回n个概率，直方块左边线的x值，及各个方块对象
    # print(n)
    # print(bins)
    # print(patches)
y1 = norm.pdf(bins1, mu1, sigma1)  # 拟合一条最佳正态分布曲线y
plt.plot(bins1, y1, 'g--')  # 绘制y的曲线


ax2.set_title('Y方向定位精度 极差=%.2f mm,均值=%.2f mm,标准差=%.2f mm,3σ=%.2f mm'%(range1,mu1,sigma1,3*sigma1))
ax2.set_xlabel('定位误差(mm)')

ax2.set_ylabel("统计概率")


#===========================================================================================================================================================
plt.suptitle('定位误差分布图',size=19)

# # print(l)
# # # # l1=[-5.79]*len(l)
# # #
# # #
# #设置图表标题，并给坐标轴加上标签
# plt.title("9月30号 15:35-10月8号 9:00 pose")
# plt.xlabel("回到站點2的次數")
# plt.ylabel("站點2橫坐標/cm")
# # # #
# # #设置刻度标记的大小
# plt.tick_params(axis='both',labelsize=15)
# # # #
# # # #
# # # #
# # # #
# # #
plt.show()
