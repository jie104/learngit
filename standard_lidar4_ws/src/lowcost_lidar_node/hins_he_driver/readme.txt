1、修改雷达参数
  方法 : 在launch文件内修改对应参数："spin_frequency_Hz", "angle_increment", "noise_filter_level"
  说明 : 雷达扫描参数将会在启动本驱动后修改为lanunch文件内的值，如要设定传感器参数如下 : 
    1.测量频率 : 100kHz
    2.扫描频率 : 15Hz
    3.噪声过滤等级 : 中等
  则launch文件内需要添加以下语句 :
    <param name="change_param" type="bool" value="true"/> 
    <param name="spin_frequency_Hz" type="int" value="10"/>
    <param name="angle_increment" type="string" value="0.025"/>
    <param name="noise_filter_level" type="string" value="2"/>
  注意：以上语句必须按照说明书等级填写，若填写错误或没有添加语句，则不改变雷达参数。
  注：本数值会覆盖windows配置软件设定的参数值，传感器断电重启后恢复windows配置软件设定的参数值。launch内传感器的ip地址和端口需要根据windows配置软件设置的值配置，否则会连接不上。
  若不需要改变雷达扫描参数，而是需要按照windows配置软件设定的参数，则"change_param"设为"false"。

2、防拖尾过滤功能
  方法 : 在launch文件内修改对应参数 : "shadows_filter_max_angle", "shadows_filter_min_angle", "shadows_filter_traverse_step", "shadows_filter_window", "shadows_filter_level"
  说明 : 以shadows_filter_traverse_step为步长，遍历雷达数据。每次搜索遍历点的后shadows_filter_window个点，若两者的垂直夹角小于shadows_filter_min_angle，或者大于shadows_filter_max_angle时，则将较长的点设置为60.001m
  1.当shadows_filter_level为-1时，按照具体参数配置过滤算法
    1)shadows_filter_max_angle : 筛选的夹角最大值
    2)shadows_filter_min_angle : 筛选的夹角最小值
    3)shadows_filter_traverse_step : 遍历点数的步长(每n个点检测一次)
    4)shadows_filter_window : 搜索窗口大小(检测遍历点的后n个值)
  2.当shadows_filter_level为0时，忽略参数设置，不增加防拖尾过滤功能
  3.当shadows_filter_level为1时，过滤速度较快，筛选角度大
  4.当shadows_filter_level为2时，过滤速度较慢，筛选角度大
  5.当shadows_filter_level为3时，过滤速度较慢，筛选角度小
  6.若launch文件没有设置以上参数，则默认如下：
    1)shadows_filter_level:-1               # 按照具体参数配置防拖尾过滤器
    2)shadows_filter_max_angle:175          # 筛选的夹角最大值为175度
    3)shadows_filter_min_angle:5            # 筛选的夹角最小值为5度
    4)shadows_filter_traverse_step:1        # 遍历每个点
    5)shadows_filter_window:2               # 搜索遍历点后2个点是否存在拖尾现象
