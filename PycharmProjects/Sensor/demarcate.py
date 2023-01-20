import os,re
import matplotlib.pyplot as plt


path = r"C:\Users\张赟\Desktop\需要求最优的数据" #文件夹目录
files= os.listdir(path) #得到文件夹下的所有文件名称


plt.rcParams['font.sans-serif'] = ['SimHei'] # 步骤一（替换sans-serif字体）
plt.rcParams['axes.unicode_minus'] = False  # 步骤二（解决坐标轴负数的负号显示问题）
l={}

for file in files:
    # print(file)
    l1,l2,l3,l4,l5,l6=[],[],[],[],[],[]
    f = open(path+'/'+file)
    iter_f=iter(f)
    for line in iter_f:
        # print(line)
        line=line.split()
        # print(line1)
        l1.append(float(line[-3]))
        l2.append(float(line[-2]))
        l3.append(float(line[-1]))
        l4.append(float(line[0]))
        l5.append(float(line[1]))
        l6.append(float(line[2]))



    print("//===============================================================================================================",file)



    #标定后的雷达机械参数
    complete_range_x=max(l1)-min(l1)
    complete_range_y=max(l2)-min(l2)
    complete_range_θ=max(l3)-min(l3)
    complete_mean_x=sum(l1)/len(l1)
    complete_mean_y=sum(l2)/len(l2)
    complete_mean_θ=sum(l3)/len(l3)

    line1 = re.split('[),_, ]', file)
    x1, x2, y1, y2, θ1, θ2 = float(line1[2]), float(line1[3]), float(line1[5]), float(line1[6]), float(line1[-3]), float(line1[-2])

    #计算权重
    optimal=1*abs((2*complete_range_x-x1-x2)/(x2-x1))+1*abs((2*complete_range_y-y1-y2)/(y2-y1))+8*abs((2*complete_range_θ-θ1-θ2)/(θ2-θ1))
    l[optimal]=file


    print('标定好的x值极差:',complete_range_x,'mm')
    print('标定好的y值极差：',complete_range_y,'mm')
    print('标定好的θ值极差：',complete_range_θ,'°')
    print('标定好的x值平均值:',complete_mean_x,'mm','   标定好的y值平均值：',complete_mean_y,'mm','   标定好的θ值平均值：',complete_mean_θ,'°')

N=100000
for num in l:
    if num<=N:
        N=num

for j in sorted(l,reverse=False):
    print('距离及角度：%s,权重最优数：%f\n'%(l[j],j))
print("最理想的距离及角度: %s,权重最优数：%f "%(l[N],N))

最理想的距离及角度: 150_+10(x_-935.000000 -135.000000 y_-38.000000 -638.000000 theta_-137.000000 -130.000000).txt,权重最优数：307.980514
