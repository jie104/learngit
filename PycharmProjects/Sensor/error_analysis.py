
import numpy as np
from scipy.stats import norm
import pandas as pd
import re
import matplotlib.pyplot as plt

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
    ax.set_title(t + r" range={:.2f}, mean={:.2f}, std={:.2f}, 3σ={:.2f} (mm)".format(r, mu, sigma, 3*sigma))
    ax.set_xlabel('error(mm)')
    ax.set_ylabel("probability")


def analysis_one_experiment(file_path):
    result = pd.read_csv(file_path)
    result.rename(columns=lambda x: x.strip(), inplace=True)
    code_id_set = set(result['code_id'])
    valid_code_ids = [code for code in code_id_set if not pd.isna(code)]
    for code_id in valid_code_ids:
        valid_data = result[result['code_id'] == code_id]
        valid_data.index = range(0, len(valid_data))
        valid_data.motion_error_x *= 1e3  # m -> mm
        valid_data.motion_error_y *= 1e3
        valid_data = valid_data.iloc[0:-1:2, :]

        fig = plt.figure(figsize=(10, 10))
        ax1 = fig.add_subplot(221)
        fit_norm(valid_data.motion_error_x, ax1, t="x")
        fig.add_subplot(222)
        plt.plot(valid_data.motion_error_x)
        ax3 = fig.add_subplot(223)
        fit_norm(valid_data.motion_error_y, ax3, t="y")
        fig.add_subplot(224)
        plt.plot(valid_data.motion_error_y)
        plt.show()




def calMean(list):
    """
    计算列表平均值，并返回
    :param list: list
    :return:
    """
    n=len(list)
    sum=0
    for x in list:
        sum+=x
    return sum/n

def calStd(list):
    """
    返回列表的标准差
    :param list:
    :return:
    """
    std=0
    n=len(list)
    mean=calMean(list)
    for x in list:
        std+=(x-mean)*(x-mean)
    return std/n
def calRnge(list):
    """
    返回列表的极差
    :param list: list
    :return:
    """
    # print(line)
    Min=min(list)
    Max=max(list)
    # print("Max:%.5f,Min:%.5f"%(Max,Min))
    return Max-Min


def drawing(error_x_list, error_y_list, ID, time_line):
    testIndex_error_x=testIndex(error_x_list)
    testIndex_error_y=testIndex(error_y_list)
    MtoMM=1000

    x_l=range(1,len(error_x_list)+1)
    plt.subplot(2, 1, 1)
    plt.title("%s,ID=%d,range=%.2f,mean=%.2f"%(time_line,ID,testIndex_error_x["range"]*MtoMM,testIndex_error_x["mean"]*MtoMM))
    plt.plot(x_l, error_x_list, c="r", label="error_x:mm")
    plt.xlabel("num")
    plt.ylabel("error_x")
    plt.legend()

    # 第二个图：折线图
    y_l=range(1,len(error_y_list)+1)
    plt.subplot(2, 1, 2)
    plt.title("%s,ID=%d,range=%.2f,mean=%.2f"%(time_line,ID,testIndex_error_y["range"]*MtoMM,testIndex_error_y["mean"]*MtoMM))
    plt.plot(y_l, error_y_list, c="r", label="error_y:mm")
    plt.xlabel("num")
    plt.ylabel("error_y")
    plt.legend()

    # # 第二个图：柱状图
    # plt.subplot(2, 4, 2)
    # X = ["苹果", "雪梨", "红浪"]
    # Y = [100, 200, 150]
    # plt.title("柱状图")
    # plt.bar(X, Y, facecolor='#9999ff', edgecolor='white')
    # plt.ylabel("Y")
    #
    # # 第三个图：条形图
    # plt.subplot(2, 4, 3)
    # plt.title("条形图")
    # plt.barh(X, Y, facecolor='#9900cc', edgecolor='white')
    #
    # # 第五个图：饼图
    # plt.subplot(2, 4, 5)
    # labels = ["香蕉", "拔辣", "西柚", "柠檬茶", "王炸"]
    # sizes = [100, 150, 30, 75, 68]
    # explode = (0, 0.1, 0, 0, 0)
    # plt.title("饼图")
    # plt.pie(sizes, explode=explode, labels=labels, autopct='%.1f%%', shadow=False, startangle=150)
    #
    # # 第六个图：散点图
    # plt.subplot(2, 4, 6)
    # X = range(0, 100)
    # Y1 = np.random.randint(0, 20, 100)
    # Y2 = np.random.randint(0, 20, 100)
    # plt.title("散点图")
    # plt.plot(X, Y1, ".", marker=".", c="#9966ff", label="Y1")
    # plt.plot(X, Y2, ".", marker="*", c="#6699ff", label="Y2")
    # plt.xlabel("X")
    # plt.ylabel("Y")
    # plt.legend(loc="upper right")
    #
    # # 第七个图：雷达图
    # plt.subplot(2, 4, 7, polar=True)
    # plt.title("雷达图", pad=20)
    # labels = np.array(["生命值", "灵敏度", "攻击力", "护甲", "守护光环", "威慑力", "成长"])
    # dataLength = 7
    # data1 = np.random.randint(5, 15, 7)
    # data2 = np.random.randint(4, 15, 7)
    # angles = np.linspace(0, 2 * np.pi, dataLength, endpoint=False)  # 分割圆周长
    # data1 = np.concatenate((data1, [data1[0]]))  # 闭合
    # data2 = np.concatenate((data2, [data2[0]]))  # 闭合
    # angles = np.concatenate((angles, [angles[0]]))  # 闭合
    # plt.polar(angles, data1, '.-', linewidth=1)  # 做极坐标系
    # plt.fill(angles, data1, alpha=0.25)  # 填充
    # plt.polar(angles, data2, '.-', linewidth=1)  # 做极坐标系
    # plt.fill(angles, data2, alpha=0.25)  # 填充
    # plt.thetagrids(angles * 180 / np.pi, labels)  # 设置网格、标签
    #
    # # 第把个图：箱线图
    # plt.subplot(2, 4, 8)
    # A = np.random.randint(0, 20, 100)
    # B = np.random.randint(5, 20, 100)
    # plt.title("箱线图")
    # plt.boxplot((A, B), labels=["A", "B"])

    plt.tight_layout(pad=1.08)
    plt.show()

def testIndex(error_list):
    testIndex_error={}
    mean_error_x=calMean(error_list)
    range_error_x=calRnge(error_list)
    testIndex_error["mean"]=mean_error_x
    testIndex_error["range"]=range_error_x
    return testIndex_error

def calSiteRange(path_file,ID):
    f = open(path_file, "r", encoding="utf-8")
    error_x_list=[]
    error_y_list=[]
    flag=True
    time_line=[]
    for line in f.readlines():
        if flag:
            print("========================",line,end="")  #不换行输出
            time_line=line
            flag=False
        line = re.split(r'[ ,\n]', line)
        # print(line)
        if len(line) >15:
            if line[-15]==str(ID):
                error_x_list.append(line[-11])
                error_y_list.append(line[-7])
    for i in range(len(error_x_list)):
        error_x_list[i]=float(error_x_list[i])
        error_y_list[i]=float(error_y_list[i])



    print("次数：%d,ID: %d"%(len(error_x_list),ID))
    print("error_x_range: %.2f mm \nerror_y_range: %.2f mm"%(calRnge(error_x_list)*1000,calRnge(error_y_list)*1000))
    print("error_x_mean: %.2f mm \nerror_y_mean: %.2f mm"%(calMean(error_x_list)*1000,calMean(error_y_list)*1000))
    drawing(error_x_list,error_y_list,ID,time_line)

def drawing_one_pictrue(file_path):
    f = open(file_path, "r", encoding="utf-8")
    list=[]
    flag=True
    for line in f.readlines():
        line = re.split(r'[ ,\n]', line)
        if flag:
            flag=False
        else:
            angle_error=float(line[1])
            list.append(angle_error)

    plt.plot(list, linewidth=0.8)
    plt.title("forward_angle_error")
    plt.xlabel("num")
    plt.ylabel("error")
    plt.tick_params(axis='both', labelsize=15)
    plt.show()


if __name__ == "__main__":

    # file1 = './pose_errors_off.txt'
    # file2 = './pose_errors_on_1.txt'
    # file3 = './pose_errors_off_2.txt'
    # file4 = './pose_errors_off_3.txt'
    # file5 = './pose_errors_on_2.txt'
    # analysis_one_experiment(file5)
    # file="/home/zxj/pose_errors/Day 26 pose_errors102020136.txt"
    # calSiteRange(file,250)
    # print("")
    # calSiteRange(file,209)
    # 未扫到码的判断可以相应的添加

    file_path="/home/zxj/data/lidar_detect/forward_angle_error.txt"
    drawing_one_pictrue(file_path)


