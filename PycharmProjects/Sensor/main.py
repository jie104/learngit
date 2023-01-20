import datetime
import time
import modbus_tk
import modbus_tk.defines as cst
import struct
import xlwt
from xlrd import open_workbook
from xlutils.copy import copy
from socket import *
from modbus_tk import modbus_tcp, hooks


def Writexls(Name, Par_rows, Par_X, Par_Y, Par_R, Par_AGV_X, Par_AGV_Y, Par_AGV_R, Station_No):  # 保存数据
    rexcel = open_workbook(Name)  # 用wlrd提供的方法读取一个excel文件
    excel = copy(rexcel)  # 用xlutils提供的copy方法将xlrd的对象转化为xlwt的对象
    table = excel.get_sheet(0)  # 用xlwt对象的方法获得要操作的sheet
    # now_time = datetime.datetime.now().strftime('%Y-%m-%d %H-%M-%S.%f')
    if Station_No == 1:
        table.write(Par_rows, 0, Par_rows)
        table.write(Par_rows, 1, Station_No)  # 测试站点
        table.write(Par_rows, 2, Par_X)  # PGV X方向
        table.write(Par_rows, 3, Par_Y)  # PGV Y方向
        table.write(Par_rows, 4, Par_R)  # PGV角度
        table.write(Par_rows, 5, Par_AGV_X)  # AGV X方向
        table.write(Par_rows, 6, Par_AGV_Y)  # AGV Y方向
        table.write(Par_rows, 7, Par_AGV_R)  # AGV 角度
    else:
        table.write(Par_rows, 9, Station_No)  # 测试站点
        table.write(Par_rows, 10, Par_X)  # PGVX方向
        table.write(Par_rows, 11, Par_Y)  # PGVY方向
        table.write(Par_rows, 12, Par_R)  # PGV角度
        table.write(Par_rows, 13, Par_AGV_X)  # AGV X方向
        table.write(Par_rows, 14, Par_AGV_Y)  # AGV Y方向
        table.write(Par_rows, 15, Par_AGV_R)  # AGV 角度
    excel.save(Name)


def ushort_to_short(u_date):  # AGV当前位置，无符号转换为有符号数
    Value0 = struct.pack("H", u_date)
    return struct.unpack("h", Value0)


AGV_IP = "192.168.33.211"
PGV_IP = "192.168.33.12"
LOGGER = modbus_tk.utils.create_logger("console")  # modbus -TCP连接
MASTER = modbus_tcp.TcpMaster(host=AGV_IP, port=502)
MASTER.set_timeout(5.0)
LOGGER.info("connected")
NavStatus = MASTER.execute(1, cst.READ_INPUT_REGISTERS, 30001, 19, data_format="")
station = 1
if NavStatus[0] == 2 and NavStatus[1] == 3:
    print("""1222""")
    NavStatus = MASTER.execute(1, cst.WRITE_MULTIPLE_REGISTERS, 40015, 1, output_value=[1])
    time.sleep(2)
    while True:
        NavStatus = MASTER.execute(1, cst.READ_INPUT_REGISTERS, 30001, 19, data_format="")
        if NavStatus[0] == 2 and NavStatus[1] == 3 and NavStatus[15] == 1 and NavStatus[14] == 1:
            now_time = datetime.datetime.now().strftime('%Y-%m-%d %H-%M-%S.%f')  # AGV状态正常，新建文件
            XlsName = "Demo " + str(AGV_IP) + "_" + str(now_time) + ".xls"
            workBook = xlwt.Workbook()  # built xls
            sht1 = workBook.add_sheet('采样数据')
            sht1.write(0, 0, "NO.")
            sht1.write(0, 1, "Station_1")
            sht1.write(0, 2, "PGV_X(mm)")
            sht1.write(0, 3, "PGV_Y(mm)")
            sht1.write(0, 4, "PGV_R(°)")
            sht1.write(0, 5, "AGV_X(mm)")
            sht1.write(0, 6, "AGV_Y(mm)")
            sht1.write(0, 7, "AGV_R(°)")
            sht1.write(0, 9, "Station_2")
            sht1.write(0, 10, "PGV_X(mm)")
            sht1.write(0, 11, "PGV_Y(mm)")
            sht1.write(0, 12, "PGV_R(°)")
            sht1.write(0, 13, "AGV_X(mm)")
            sht1.write(0, 14, "AGV_Y(mm)")
            sht1.write(0, 15, "AGV_R(°)")
            workBook.save(XlsName)
            a = ushort_to_short(NavStatus[3])
            b = ushort_to_short(NavStatus[5])
            c = ushort_to_short(NavStatus[7])
            AGV_Xpos0 = round(a[0] / 1000, 4) * 1000
            AGV_Ypos0 = round(b[0] / 1000, 4) * 1000
            AGV_Rpos0 = round(c[0] / 1000 * 180 / 3.1415926, 2)
            # if AGV_Rpos0 < 0 :
            # AGV_Rpos0 = AGV_Rpos0 + 360
            time.sleep(2)
            break
        else:
            time.sleep(1)
host = "localhost"
bufsiz = 1024
rows = 0
request_cmd_left = b'\xe8\x17'  # 请求命令，配置模式
request_cmd = b'\xc8\x37'  # 请求命令，读位置信息
tcpCliSock = socket(AF_INET, SOCK_STREAM)  # 开启套接字
tcpCliSock.connect((PGV_IP, 4096))

tcpCliSock.send(request_cmd_left)  # 配置PGV
print("""1223""")
response = tcpCliSock.recv(bufsiz)
print(response)
time.sleep(1)
add00 = 0
while add00 <= 10000:
    add00 = add00 + 1
    tcpCliSock.send(request_cmd)  # 发送读取位置信息
    response = tcpCliSock.recv(bufsiz)  # 接受返回信息
    # print(response)
    if not response:
        break
    if len(response) == 21:
        new = struct.unpack('<bbbbbbbbbbbbbbbbbbbbb', bytes(response))  # 转换为字节
        # print(new)
        if new[0] & 0x03 == 0:
            xPos = (new[2] & 0x07) * 0x80 * 0x4000 + new[3] * 0x4000 + new[4] * 0x80 + new[5]  # 字节2~5转换为X方向坐标
            if xPos > 0x7fffff:
                xPos = (xPos - 0xffffff) / 10
            else:
                xPos = xPos / 10
            yPos = new[6] * 0x80 + new[7]  # 字节6~7转换为Y方向坐标
            if yPos > 0x2000:
                yPos = (yPos - 0x4000) / 10
            else:
                yPos = yPos / 10
            Rod = new[10] * 0x80 + new[11]  # 字节10~11转换为角度值
            Rod = Rod / 10
            print(add00, "       ", station, "  ", xPos, yPos, Rod, AGV_Xpos0, AGV_Ypos0, AGV_Rpos0)
            if add00 == 1:
                MAX_xPos = xPos
                MAX_yPos = yPos
                MAX_Rod = Rod
                MAX_AGV_Xpos0 = AGV_Xpos0
                MAX_AGV_Ypos0 = AGV_Ypos0
                MAX_AGV_Rpos0 = AGV_Rpos0
                Min_xPos = xPos
                Min_yPos = yPos
                Min_Rod = Rod
                Min_AGV_Xpos0 = AGV_Xpos0
                Min_AGV_Ypos0 = AGV_Ypos0
                Min_AGV_Rpos0 = AGV_Rpos0

            MAX_xPos = max(MAX_xPos, xPos)
            MAX_yPos = max(MAX_yPos, yPos)
            MAX_Rod = max(MAX_Rod, Rod)
            MAX_AGV_Xpos0 = max(MAX_AGV_Xpos0, AGV_Xpos0)
            MAX_AGV_Ypos0 = max(MAX_AGV_Ypos0, AGV_Ypos0)
            MAX_AGV_Rpos0 = max(MAX_AGV_Rpos0, AGV_Rpos0)
            Min_xPos = min(Min_xPos, xPos)
            Min_yPos = min(Min_yPos, yPos)
            Min_Rod = min(Min_Rod, Rod)
            Min_AGV_Xpos0 = min(Min_AGV_Xpos0, AGV_Xpos0)
            Min_AGV_Ypos0 = min(Min_AGV_Ypos0, AGV_Ypos0)
            Min_AGV_Rpos0 = min(Min_AGV_Rpos0, AGV_Rpos0)

            # 保存数据
            if station == 1:
                rows = rows + 1
                Writexls(XlsName, rows, xPos, yPos, Rod, AGV_Xpos0, AGV_Ypos0, AGV_Rpos0, station)
                station = 2
            else:
                Writexls(XlsName, rows, xPos, yPos, Rod, AGV_Xpos0, AGV_Ypos0, AGV_Rpos0, station)
                station = 1
            if add00 == 300:
                Writexls(XlsName, rows + 1, MAX_xPos, MAX_yPos, MAX_Rod, MAX_AGV_Xpos0, MAX_AGV_Ypos0, MAX_AGV_Rpos0,
                         station)
                Writexls(XlsName, rows + 2, Min_xPos, Min_yPos, Min_Rod, Min_AGV_Xpos0, Min_AGV_Ypos0, Min_AGV_Rpos0,
                         station)
        else:
            print("PGV Error")

    NavStatus = MASTER.execute(1, cst.WRITE_MULTIPLE_REGISTERS, 40015, 1, output_value=[station])
    time.sleep(2)
    while True:
        NavStatus = MASTER.execute(1, cst.READ_INPUT_REGISTERS, 30001, 19, data_format="")
        if NavStatus[0] == 2 and NavStatus[1] == 3 and NavStatus[15] == 1 and NavStatus[14] == station:
            time.sleep(5)
            a = ushort_to_short(NavStatus[3])
            b = ushort_to_short(NavStatus[5])
            c = ushort_to_short(NavStatus[7])
            AGV_Xpos0 = round(a[0] / 1000, 4) * 1000
            AGV_Ypos0 = round(b[0] / 1000, 4) * 1000
            AGV_Rpos0 = round(c[0] / 1000 * 180 / 3.1415926, 2)
            # if AGV_Rpos0 < 0 :
            # AGV_Rpos0 = AGV_Rpos0 + 360
            break
        else:
            time.sleep(0.5)
    time.sleep(1)
tcpCliSock.close()
