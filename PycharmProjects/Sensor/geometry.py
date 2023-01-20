import matplotlib.pyplot as plt
import csv

plt.rcParams['font.sans-serif'] = ['SimHei'] # 步骤一（替换sans-serif字体）
plt.rcParams['axes.unicode_minus'] = False  # 步骤二（解决坐标轴负数的负号显示问题）

#从文件中提取数据
l=['运动补偿.csv','无补偿.csv','高精度.csv']
for filename in l:
    with open(filename) as f:
        reader=csv.reader(f)

        data=[]
        for i in range(50):
            x=next(reader)
        for k in range(1800):
            x=next(reader)
            data+=x

    for j,x in enumerate(data):
        data[j]=float(x)

    #围成的面积
    S1,S2=0,0
    for i in range(len(data)-1):
        S1+=max(data[i],data[i+1])
        S2+=min(data[i],data[i+1])
    S=(S1+S2)/2
    print(filename[:-4],'围成的面积：',S)



    #控制图像颜色plt.scatter(x_value,data,c=data,cmap=plt.cm.Blues,edgecolor='none',s=40)
    plt.plot(data,linewidth=0.8,label=filename[:-4])


    #设置图表标题，并给坐标轴加上标签
    plt.title("原地旋转定位效果对比")
    plt.xlabel("周期")
    plt.ylabel("定位精度")

    #设置刻度标记的大小
    plt.tick_params(axis='both',labelsize=15)




    plt.legend()
plt.show()
