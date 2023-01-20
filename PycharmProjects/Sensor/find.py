import sys,re,csv
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei'] # 步骤一（替换sans-serif字体）
plt.rcParams['axes.unicode_minus'] = False  # 步骤二（解决坐标轴负数的负号显示问题）
l=[]
l2=[]
l3=[]
flag=False
flag1=False
# list=['sros22.txt','sros23.txt','sros24.txt','sros25.txt','sros26.txt','sros27.txt','sros28.txt','sros29.txt','sros30.txt',
#       'sros31.txt','sros32.txt','sros33.txt','sros34.txt']
list=['uup.txt']
for x in list:
    f = open(x,"r",encoding="utf-8")
    for line in f.readlines():


#绘制二位码扫到的横坐标--------------------------------------------------------------------------------------------------------------------

#         a=''
#         x1=0
#         key = "th register changed!"
#         if key in line:
#             l1=[]
#             # print(line)
#         # s = re.findall('"TimeSpan":"([\d.]+)"', line)
#         # print("***", line)
#             if 'SRC the 1th register changed!' in line:
#                 if float(line[-30:].split()[-1]) < -57000 and float(line[-30:].split()[-1])*float(line[-30:].split()[-3])<0:
#                     flag=True
#                     l3.append(float(line[-30:].split()[-1]))
#                 else:
#                     flag=False
#             if 'SRC the 8th register changed!' in line:
#                 # print(line)
#                 if flag:
#                     for s in line[-30:].split():
#                     # print(line[-30:].split())
#                         if s[0] in '-0123456789':
#                             l1.append(float(s))
#                     # print(l1)
#                     if l1[0] < l1[1]:
#                          # print(l1)
#                         if l1[1]==0:
#                             print(line)
#                             l1[1]=100000
#                         l.append(l1[1]/10000)
#                             # if l1[1]>60000 :
#                             #      print(line)
#                         # else:
#                         #     print(line)
#
# # plt.plot(l3,linewidth=0.8)
# plt.plot(l, linewidth=0.8)
# plt.title("9月30号 15:35-10月8号 9:00 二维码扫描")
# plt.xlabel("回到站點3的次數")
# plt.ylabel("站點3橫坐標/cm")
# plt.tick_params(axis='both', labelsize=15)
# plt.show()


#绘制agv位姿横坐标---------------------------------------------------------------------------------------------------------------


#         a1=''
#         key1='Navigation 彻底结束，目标站点：2 当前位置：Pose(0'
#         if key1 in line:
#             # for s1 in line[-30:].split():
#             for s1 in line[-30:].split()[0]:
#                 if s1=='-':
#                     flag1=True
#
#                 if s1==',':
#                     l2.append(float(a1)*100)
#                     flag1=False
#                     break
#                 if flag1:
#                     a1 += s1
# print(l2)


# print(l)
# # # l1=[-5.79]*len(l)
#繪製圖形
plt.plot(l2,linewidth=0.8)
# #
# #
#设置图表标题，并给坐标轴加上标签
plt.title("9月30号 15:35-10月8号 9:00 pose")
plt.xlabel("回到站點3的次數")
plt.ylabel("站點3橫坐標/cm")
# #
#设置刻度标记的大小
plt.tick_params(axis='both',labelsize=15)
# #
# #
# #
# #
# #
plt.show()



#-------------------------------------------------------------------------------------------------------------------------------

f.close()