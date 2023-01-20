import sys,re,csv

import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei'] # 步骤一（替换sans-serif字体）
plt.rcParams['axes.unicode_minus'] = False  # 步骤二（解决坐标轴负数的负号显示问题）
# list=['sros22.txt','sros23.txt','sros24.txt','sros25.txt','sros26.txt','sros27.txt','sros28.txt','sros29.txt','sros30.txt',
#       'sros31.txt','sros32.txt','sros33.txt','sros34.txt']
list=['peckfuch.day_pose_errors.txt']
l1=[]
l2=[]
l3=[]
key1 = '001900DH002073'
key2='001900DH002077'
flag=False
k=0
l=0
sum_x=sum_y=0

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

mean_x=sum_x/(len(l1)-l)
mean_y=sum_y/(len(l2)-l)

num1=[sum_x/(len(l1)-l)]*len(l1)
num2=[sum_y/(len(l2)-l)]*len(l2)

print(l)
print(l1)
print(l2)
print(l3)


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
ax1.plot(l1)
ax1.plot(num1)

# ax1.set_title('pose_x',fontsize=14)
ax1.set_xlabel('二维码 '+key1+' '+' 平均值：'+str(mean_x))
ax1.set_ylabel("pose_x\m")
# plt.plot(l1,linewidth=0.8)
# plt.plot(l2,linewidth=0.8)
# plt.plot(l3,linewidth=0.8)

ax2=fig.add_subplot(212)
ax2.plot(l2,label='未标定')
ax2.plot(num2)

# ax2.set_title('pose_y',fontsize=14)
ax2.set_xlabel('二维码 '+key1+' 平均值：'+str(mean_y))
ax2.set_ylabel("pose_y\m")

# ax3=fig.add_subplot(212)
# ax3.plot(l3)


# ax3.set_title('pose_θ',fontsize=14)
# ax3.set_xlabel('二维码ID '+key1)
# ax3.set_ylabel("pose_θ\°")

plt.suptitle('12-16至12-17 pepperfuch600ul '+'扫不到码共'+str(l)+'次'+',码扫不到时,x和y默认输出0.03')

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