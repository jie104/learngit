import sys,re,csv

import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei'] # 步骤一（替换sans-serif字体）
plt.rcParams['axes.unicode_minus'] = False  # 步骤二（解决坐标轴负数的负号显示问题）
# list=['sros22.txt','sros23.txt','sros24.txt','sros25.txt','sros26.txt','sros27.txt','sros28.txt','sros29.txt','sros30.txt',
#       'sros31.txt','sros32.txt','sros33.txt','sros34.txt']
list=['仿真数据.txt']
l1=[]
l2=[]
l3=[]
key1 = 'Navigation 彻底结束，目标站点：2 当前位置：Pose('
flag=False
for x in list:
    f = open(x,"r",encoding="utf-8")
    for line in f.readlines():
        if key1 in line:
            print(line)
            line=re.split('[,,(,)]',line[-32:])
            l1.append(float(line[1]))
            l2.append(float(line[2]))
            l3.append(float(line[3]))
            # if float(line[2]) >0.235:
            #     print(line)
print(l1)
print(l2)
print(l3)


            # print(line)
            # num = ''
            # for y in line[-35:]:
            # # print(line[-35:])
            #     if y == ',':
            #         flag = False
            #         l1.append(float(num))
            #         break
            #
            #     if flag:
            #         num += y
            #
            #     if y=='(':
            #         flag=True


# #繪製圖形
fig=plt.figure()
# plt.title("10.31 15:13-11.3 13:46 昆山18小车pose")

ax1=fig.add_subplot(221)
ax1.plot(l1)

# ax1.set_title('pose_x',fontsize=14)
ax1.set_xlabel("到站点2的次數")
ax1.set_ylabel("pose_x\m")
# plt.plot(l1,linewidth=0.8)
# plt.plot(l2,linewidth=0.8)
# plt.plot(l3,linewidth=0.8)

ax2=fig.add_subplot(222)
ax2.plot(l2,label='未标定')


# ax2.set_title('pose_y',fontsize=14)
ax2.set_xlabel("到站点2的次數")
ax2.set_ylabel("pose_y\m")

ax3=fig.add_subplot(212)
ax3.plot(l3)


# ax3.set_title('pose_θ',fontsize=14)
ax3.set_xlabel("到站点2的次數")
ax3.set_ylabel("pose_θ\°")

plt.suptitle('11.1 19:42-11.3 11:05 600C WANJI')

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