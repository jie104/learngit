import sys,re,csv

import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei'] # 步骤一（替换sans-serif字体）
plt.rcParams['axes.unicode_minus'] = False  # 步骤二（解决坐标轴负数的负号显示问题）
# list=['sros22.txt','sros23.txt','sros24.txt','sros25.txt','sros26.txt','sros27.txt','sros28.txt','sros29.txt','sros30.txt',
#       'sros31.txt','sros32.txt','sros33.txt','sros34.txt']
list=['test1.txt']
l1=[]
l2=[]
l3=[]

num1=float(input("请输入标定的x:"))
num2=float(input("请输入标定的y:"))
num3=float(input("请输入标定的θ:"))


for x in list:
    f = open(x,"r",encoding="utf-8")
    for line in f.readlines():
            # print(line)
        line = re.split('[:, ]', line)
        # print(line)
        l1.append(abs(float(line[1])-num1))
        l2.append(abs(float(line[3])-num2))
        l3.append(abs(float(line[5])-num3))
            # if float(line[2]) >0.235:
            #     print(line)
#
print(l1)
print(l2)
print(l3)
#

# #繪製圖形
fig=plt.figure()
# plt.title("10.31 15:13-11.3 13:46 昆山18小车pose")

ax1=fig.add_subplot(221)
ax1.plot(l1)
# ax1.plot([num1]*len(l1))

# ax1.set_title('pose_x',fontsize=14)
ax1.set_xlabel('次数')
ax1.set_ylabel("pose_x_error\m")
# plt.plot(l1,linewidth=0.8)
# plt.plot(l2,linewidth=0.8)
# plt.plot(l3,linewidth=0.8)

ax2=fig.add_subplot(222)
ax2.plot(l2,label='未标定')
# ax2.plot([num2]*len(l2))

# ax2.set_title('pose_y',fontsize=14)
ax2.set_xlabel('次数')
ax2.set_ylabel("pose_y_error\m")

ax3=fig.add_subplot(212)
ax3.plot(l3)
# ax3.plot([num3]*len(l3))

# ax3.set_title('pose_θ',fontsize=14)
ax3.set_xlabel('次数')
ax3.set_ylabel("pose_θ_error\°")

plt.suptitle('误差')

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