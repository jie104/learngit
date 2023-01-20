import sys,re,csv
import matplotlib.pyplot as plt


plt.rcParams['font.sans-serif'] = ['SimHei'] # 步骤一（替换sans-serif字体）
plt.rcParams['axes.unicode_minus'] = False  # 步骤二（解决坐标轴负数的负号显示问题）
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
    range_x=max(l1)-min(l1)
    range_y=max(l2)-min(l2)
    range_θ=max(l3)-min(l3)
    mean_x=sum(l1)/len(l1)
    mean_y=sum(l2)/len(l2)
    mean_θ=sum(l3)/len(l3)

    print("//-------------------------------------------------------",x)
    print('标定好的x值极差:',range_x,'mm')
    print('标定好的y值极差：',range_y,'mm')
    print('标定好的θ值极差：',range_θ,'°')
    print('标定好的x值平均值:',mean_x,'mm','   标定好的y值平均值：',mean_y,'mm','   标定好的θ值平均值：',mean_θ,'°')


    # #繪製圖形
    fig=plt.figure()
    plt.title(x)

    ax1=fig.add_subplot(221)
    ax1.plot(l1)
    ax1.plot(l4)
    ax1.plot([mean_x]*len(l1))

    ax1.set_title('雷达机械参数x',fontsize=14)
    ax1.set_xlabel("测试次數")
    ax1.set_ylabel("机械参数x")
        # plt.plot(l1,linewidth=0.8)
        # plt.plot(l2,linewidth=0.8)
        # plt.plot(l3,linewidth=0.8)

    ax2=fig.add_subplot(222)
    ax2.plot(l2,label='未标定')
    ax2.plot(l5,label='已标定')
    ax2.plot([mean_y]*len(l2))


    ax2.set_title('雷达机械参数y',fontsize=14)
    ax2.set_xlabel("测试次數")
    ax2.set_ylabel("机械参数y")

    ax3=fig.add_subplot(212)
    ax3.plot(l3)
    ax3.plot(l6)
    ax3.plot([mean_θ]*len(l3))


    ax3.set_title('雷达机械参数θ',fontsize=14)
    ax3.set_xlabel("测试次數")
    ax3.set_ylabel("机械参数θ")

# #设置图表标题，并给坐标轴加上标签
# plt.title("标定后的雷达机械参数")
# plt.xlabel("测试次數")
# plt.ylabel("机械参数x")
# # # #
# # #设置刻度标记的大小
# plt.tick_params(axis='both',labelsize=15)
# # # #
#
    plt.show()




    f.close()