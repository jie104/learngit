import sys,re,csv

import matplotlib.pyplot as plt
import numpy as np
from math import sqrt
import matplotlib.mlab as mlab
# from mpmath import norm

plt.rcParams['font.sans-serif'] = ['SimHei'] # 步骤一（替换sans-serif字体）
plt.rcParams['axes.unicode_minus'] = False  # 步骤二（解决坐标轴负数的负号显示问题）
# list=['sros22.txt','sros23.txt','sros24.txt','sros25.txt','sros26.txt','sros27.txt','sros28.txt','sros29.txt','sros30.txt',
#       'sros31.txt','sros32.txt','sros33.txt','sros34.txt']



list=['pepperfuch1.day_pose_errors.txt']
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
#pose平均值
mean_x=sum_x/len(l1)
mean_y=sum_y/len(l2)
print(mean_x)
print(mean_y)

#计算误差
for i in range(len(l1)):
    l1[i]=(l1[i]-mean_x)
    l2[i]=(l2[i]-mean_y)

print(l1)
print(l2)
#pose_error平均值
error_mean_x=sum(l1)/(len(l1)-l)
error_mean_y=sum(l2)/(len(l2)-l)
print(error_mean_x)
print(error_mean_y)

num1=[sum_x/(len(l1)-l)]*len(l1)
num2=[sum_y/(len(l2)-l)]*len(l2)


#极差
range_x=(max(l1)-min(l1))
range_y=(max(l2)-min(l2))


#方差
Var_x=Var_y=0
for x in l1:
   Var_x+=(x-error_mean_x)**2
standard_dev_x=sqrt(Var_x/len(l1))

for y in l2:
   Var_y+=(y-error_mean_y)**2
standard_dev_y=sqrt(Var_y/len(l2))


#计算误差
for i in range(len(l1)):
    l1[i]=l1[i]*1000
    l2[i]=l2[i]*1000


# print(l)
# print(l1)
# print(l2)
# print(l3)


#             print(line)
#             num = ''
#             for y in line[-35:]:
#             # print(line[-35:])
#                 if y == ',':
#                     flag = False
#                     l1.append(float(num))
#                     break
#
#                 if flag:
#                     num += y
#
#                 if y=='(':
#                     flag=True
#
#
# #繪製圖形
fig=plt.figure()
# plt.title("10.31 15:13-11.3 13:46 昆山18小车pose")

ax1=fig.add_subplot(211)
# ax1.plot(l1)
ax1.plot(label='123')

#曲线图
# ax1.plot(l1)
# ax1.plot(num1)

#直方图
group_num = 25  # 直方图组数
a=plt.hist(l1, np.linspace(min(l1), max(l1), group_num), histtype='bar', rwidth=0.8)

# b=mlab.normpdf(a,1,2)
# plt.plot(a,b,'r--')

ax1.set_title('pose_x误差分布图',fontsize=14)
ax1.set_xlabel('二维码 '+key1+' '+' pose_x误差(单位：mm)')

ax1.set_ylabel("次数")
# plt.plot(l1,linewidth=0.8)
# plt.plot(l2,linewidth=0.8
# plt.plot(l3,linewidth=0.8)

ax2=fig.add_subplot(212)
# ax2.plot(l2,label='未标定')
# ax2.plot(num2)

#直方图
plt.hist(l2, np.linspace(min(l2), max(l2), group_num), histtype='bar', rwidth=0.8)
# l3=range(min(l2),max(l2),1)
# ax2.xticks(l3)

ax2.set_title('pose_y误差分布图',fontsize=14)
ax2.set_xlabel('二维码 '+key1+' pose_y误差(单位：mm)')
ax2.set_ylabel("次数")

# ax3=fig.add_subplot(212)
# ax3.plot(l3)


# ax3.set_title('pose_θ',fontsize=14)
# ax3.set_xlabel('二维码ID '+key1)
# ax3.set_ylabel("pose_θ\°")

plt.suptitle('12-16至12-17 倍加福雷达600ul '+'扫不到码共'+str(l)+'次'+',码扫不到时,pose_x和pose_y默认输出0.03'+'\n'+'\n'
             +'error_pose_x极差：'+str(("%.2f"%(range_x*1000)))+'mm '+',error_pose_x标准差：'+str(("%.2f"%(standard_dev_x*1000)))+'mm,'+'3σ：'+str(("%.2f"%(3*standard_dev_x*1000)))+'mm'+'\n'
             +'error_pose_y极差：'+str(("%.2f"%(range_y*1000)))+'mm '+',error_pose_y标准差：'+str(("%.2f"%(standard_dev_y*1000)))+'mm,'+'3σ：'+str(("%.2f"%(3*standard_dev_y*1000)))+'mm')

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
#
#
#
# #-------------------------------------------------------------------------------------------------------------------------------
#
f.close()