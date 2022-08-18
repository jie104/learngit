# sphinx_gallery_thumbnail_number = 3
import sys
sys.path.append('../communication')
import matplotlib.pyplot as plt
import numpy as np
import monitor_pb2
from datetime import datetime, timedelta
import os

# import urllib5
#
# f = urllib5.urlopen("192.168.80.212/api/v0/file/get?file=")
# with open("demo2.zip", "wb") as code:
#     code.write(f.read())

x = []
y_battery_remain = []
y_battery_current = []
y_battery_current_mean = []
y_raw_remain_time = []  # 预计使用时间 t = C / I
y_remain_time_1 = []  # 预计使用时间 t = C / (I^1.2)
y_remain_time_2 = []  # 预计使用时间 t = C / (I^1.2) recent_mean_current
y_battery_voltage = [] # 电压
y_battery_power = [] # 功率

x_slop_of_battery_remain = []  # 电池剩余容量百分比的斜率，(c1-c2)/t
y_slop_of_battery_remain = []  # 电池剩余容量百分比的斜率，(c1-c2)/t
last_battery_remain_change_time = None  # 上一个电池剩余容量百分比变化的时间
last_battery_remain = 0  # 上一个电池剩余容量百分比

recent_battery_current_list = [] # 最近的电流列表

dir_path = '/home/john/project/sros/monitor_data/117_all/monitor/'
mf_list = os.listdir(dir_path)
mf_list.sort(key=lambda x: int(x[2:-3]))
# print(mf_list)

for file_name in mf_list:

    # 过滤掉时间不对的情况
    record_file_time = datetime.fromtimestamp(int(file_name[2:-3]))
    if record_file_time < datetime(2019, 5, 19):
        continue

    with open(dir_path + file_name, 'rb') as proto_file:

        size_bytes = proto_file.read(2)
        size = int.from_bytes(size_bytes, 'big', signed=False)
        # print(size)

        data = proto_file.read(size)
        # print(data)
        header = monitor_pb2.Header()
        header.ParseFromString(data)
        while (True):
            size_bytes = proto_file.read(2)
            size = int.from_bytes(size_bytes, 'big', signed=False)
            # print(size)
            if size == 0:
                break

            data = proto_file.read(size)
            # print(data)
            record = monitor_pb2.Record()
            try:
                record.ParseFromString(data)
            except:
                continue

            record_time = datetime.fromtimestamp(record.timestamp / 1000)
            battery_remain_percentage = record.hardware_state.battery_remain_percentage / 10

            # 过滤掉不正确的数据
            if record_time < datetime(2019, 1,
                                      1) or battery_remain_percentage > 100 or record.hardware_state.battery_current == 0:
                continue

            # 电池剩余电量的斜率
            if last_battery_remain_change_time:
                if last_battery_remain != battery_remain_percentage and last_battery_remain_change_time != record_time:
                    k = (last_battery_remain - battery_remain_percentage) / (
                            last_battery_remain_change_time.timestamp() - record_time.timestamp())
                    x_slop_of_battery_remain.append(record_time)
                    y_slop_of_battery_remain.append(k * 10000 * -1)
                    # print(k)
                    last_battery_remain_change_time = record_time
                    last_battery_remain = battery_remain_percentage
            else:
                last_battery_remain_change_time = record_time
                last_battery_remain = battery_remain_percentage

            y_battery_remain.append(battery_remain_percentage)
            y_battery_current.append(record.hardware_state.battery_current / 100)

            raw_remain_time_hour = 30 / (record.hardware_state.battery_current / 1000)  # 剩余使用时间（小时）
            y_raw_remain_time.append(raw_remain_time_hour);

            remain_time_hour = 30 / pow((record.hardware_state.battery_current / 1000), 1.2)  # 剩余使用时间（小时）
            y_remain_time_1.append(remain_time_hour);

            x.append(record_time)

            # end_time = datetime(2019, 5, 28, 17, 11)
            # end_time = datetime(2019, 5, 23, 7, 11)
            end_time = datetime(2019, 5, 25, 8, 0)
            recent_battery_current_list.append(record.hardware_state.battery_current)
            if len(recent_battery_current_list) > 60:
                recent_battery_current_list.pop(0)
            recent_mean_current = np.mean(recent_battery_current_list)
            y_battery_current_mean.append(recent_mean_current / 100)
            n = 1.1
            R = 10
            remain_time_hour = R * pow((30 / R), n) * battery_remain_percentage / 100  / pow((recent_mean_current / 1000), n) - (end_time-record_time).total_seconds() / 3600 + 0 # 剩余使用时间（小时）
            y_remain_time_2.append(remain_time_hour);

            # y_battery_voltage.append(record.hardware_state.battery_velotage / 1000)
            # y_battery_power.append(record.hardware_state.battery_velotage / 1000 * record.hardware_state.battery_current / 1000 /2)


# x = np.arange(0, 10, 0.2)
# y = np.sin(x)
fig, ax = plt.subplots()
ax.plot(x, y_battery_current, 'C1', label='battery_current')
ax.plot(x, y_battery_remain, 'C2', label='battery_remain')
# ax.plot(x, y_raw_remain_time, 'C3', label='raw_remain_time t=C/I')
# ax.plot(x, y_remain_time_1, 'C5', label='remain_time_1 t=C/(I^1.2)')
ax.plot(x, y_remain_time_2, 'C6', label='remain_time_1 t=C/(I^1.2)，recent_mean_current')
# ax.plot(x, y_battery_power, 'C9', label='battery_power')
ax.plot(x, y_battery_current_mean, 'C7', label='mean_current')
# ax.plot(x, y_battery_voltage, 'C8', label='battery_voltage')

# ax.plot(x_slop_of_battery_remain, y_slop_of_battery_remain, 'C4', label='slop_of_battery_remain')

ax.legend()

# plt.xlim(datetime(2019, 5, 28, 3), datetime(2019, 5, 28, 18))
# plt.xlim(datetime(2019, 5, 22, 9), datetime(2019, 5, 23, 8))
# plt.xlim(datetime(2019, 5, 25, 1), datetime(2019, 5, 25, 9))
plt.ylim(0, 100)

plt.show()
