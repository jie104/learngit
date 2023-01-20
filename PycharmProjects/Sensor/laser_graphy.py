# coding:utf-8
import re
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import norm


def fit_norm(values, ax, group_num=50, t=""):

    r = max(values) - min(values)
    mu = np.mean(values)
    sigma = np.std(values)

    tumor_min_1 = min(values)
    tumor_max_1 = max(values)

    n, bins, patches = plt.hist(values, group_num, density=True, edgecolor="black", facecolor='gray', alpha=0.6,
                                range=[int(tumor_min_1), int(tumor_max_1)])
    y = norm.pdf(bins, mu, sigma)
    plt.plot(bins, y, 'g--')
    ax.set_title(t + r" range={:.2f}, mean={:.2f}, std={:.2f}, 3σ={:.2f} (°)".format(r, mu, sigma, 3*sigma))
    ax.set_xlabel('error(°)')
    ax.set_ylabel("probability")


def str2float(List):
    for i in range(len(List)):
        List[i]=float(List[i])



def drawCurve(List1,List2,lowcost_topic):
    # 解决中文显示问题
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.rcParams['axes.unicode_minus'] = False
    str2float(List1)
    str2float(List2)

    N=len(List1)
    M=len(List2)
    thresh=10

    # 生成figure对象
    fig = plt.figure()
    # 生成axes对象
    axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])

    x1 = np.linspace(0, N+thresh, N)
    x2 = np.linspace(0, M+thresh, M)
    # y = np.power(np.e, x)
    # y1 = np.sin(x) * 10 + 30
    # 绘制散点
    axes.plot(x1, List1, c="green", label="pepperl", ls='-.', alpha=0.6, lw=2, zorder=2)
    axes.plot(x2, List2, c="blue", label=lowcost_topic, ls=':', alpha=1, lw=1, zorder=1)
    # 设置图像标题
    axes.set_title("Forward Line Angle")

    axes.legend()
    # 显示图像
    plt.show()



def drawing(path_file):
    f = open(path_file, "r", encoding="utf-8")
    flag=True
    lowCostList=[]
    pepperlList=[]
    i=0
    for line in f.readlines():
        line = re.split(r'[ ,\n]', line)
        if i==0:
            i+=1
            continue

        if line[0]!="/r2000_node/scan" and flag:
            lowcost_topic=line[0]
            flag=False
        if line[0]==lowcost_topic:
            lowCostList.append(line[1])
        else:
            pepperlList.append(line[1])

        print(line[1])
        # drawCurve(pepperlList,lowCostList,lowcost_topic)

    str2float(lowCostList)
    str2float(pepperlList)

    fig = plt.figure(figsize=(10, 10))
    ax1 = fig.add_subplot(221)
    fit_norm(pepperlList, ax1, t="pepperl")
    fig.add_subplot(222)
    plt.plot(pepperlList)
    ax3 = fig.add_subplot(223)
    fit_norm(lowCostList, ax3, t=lowcost_topic)
    fig.add_subplot(224)
    plt.plot(lowCostList)
    plt.show()








if __name__ == "__main__":
    path_file= "/home/zxj/data/lidar_detect/forward_angle_error.txt"
    drawing(path_file)

