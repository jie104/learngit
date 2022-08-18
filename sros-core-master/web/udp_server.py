# -*- coding: utf-8 -*-
import fcntl
import socket
import json
import struct
import threading
from BaseHTTPServer import BaseHTTPRequestHandler
from BaseHTTPServer import HTTPServer
import cgi
import os
from db_util import *
import time
#import psutil
import sros_log

import logging

_logger = logging.getLogger(__name__)

IP_FILE_PATH = "/tmp/nodes_client_nat_ip"
NETWORK = '<broadcast>'
IP_BYTES_LEN = 15


def get_mac_address(if_name):
    try:
        _logger.info("get_mac_address" + str(if_name))
        mac = open('/sys/class/net/' + if_name + '/address').readline().strip().upper()
    except:
        mac = "00:00:00:00:00:00"

    return mac


# import fcntl
# def get_MAC_address(if_name):
#     s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     try:
#         info = fcntl.ioctl(s.fileno(), 0x8927,  struct.pack('256s', if_name[:15]))
#         return ':'.join(['%02X' % ord(char) for char in info[18:24]])
#     except IOError:
#         return "00:00:00:00:00:00"


class UdpInfoServer(object):
    def __init__(self, host='', port=9999, head_code=b'STD'):
        self.head_code = head_code
        self.host = host
        self.port = port
        self.fmt = ""
        self.min_buffer_size = 0
        self.socket = None
        self.db = DBUtil()
        if self.head_code:
            self.set_fmt()

    def set_fmt(self):
        self.fmt = ">{0}sI".format(len(self.head_code))
        self.min_buffer_size = len(self.head_code) + 4

    def initialize(self):
        if self.socket is None:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            try:
                self.socket.bind((self.host, self.port))
            except socket.error:
                _logger.error(socket.error)
                raise socket.error

    def run(self):
        try:
            self.initialize()
        except socket.error as e:
            _logger.error(e)
            raise socket.error
        else:
            while 1:
                try:
                    data_to_decode, endpoint = self.socket.recvfrom(1024)
                    print(data_to_decode)
                    _logger.info("data_to_decode" + str(data_to_decode))

                    if len(data_to_decode) < self.min_buffer_size:
                        return -1
                    else:
                        try:
                            print(self.fmt, data_to_decode[0:self.min_buffer_size])
                            head_code, pack_len = struct.unpack(self.fmt, data_to_decode[0:self.min_buffer_size])
                        except Exception as e:
                            print('fuck')
                            print(e)
                            _logger.error(e)
                        else:
                            self.send_vehicle_info(endpoint)
                except KeyboardInterrupt:
                    break

    def get_cpu_id(self):
        try:
            with open('/sys/bus/soc/devices/soc0/soc_id') as f:
                cpu_ip = f.readline().strip()
        except IOError:
            cpu_ip = self.get_comm_interface_mac_addr()

        return cpu_ip

    def get_comm_interface_mac_addr(self):
        comm_interface = self.db.get_config_value("network.communication_interface")

        mac_addr = "00:00:00:00:00:00"
        if comm_interface == "enp3s0":
            mac_addr = get_mac_address("enp3s0")
            if mac_addr == "00:00:00:00:00:00":
                mac_addr = get_mac_address("enp1s0")
        else:
            mac_addr = get_mac_address(comm_interface)

        return mac_addr

    def get_vehicle_info(self):
        data = dict()
        data["nickname"] = self.db.get_config_value("main.nickname")
        data["serial_no"] = self.get_cpu_id()
        data["fw_version"] = "NA"
        data["hw_version"] = "NA"
        data["vehicle_serial_no"] = self.db.get_config_value("main.vehicle_serial_no")
        data["vehicle_type"] = self.db.get_config_value("main.vehicle_type")
        data["vehicle_action_unit"] = self.db.get_config_value("main.action_unit")
        data["mac_address"] = self.get_comm_interface_mac_addr()
        data["ip"] = self.get_real_ip()
        # print(data)
        _logger.info("get_vehicle_info" + str(data))

        return data

    def send_vehicle_info(self, endpoint):
        data_to_send = self.get_vehicle_info()
        data_to_send_str = json.dumps(data_to_send)

        fmt = self.fmt + "{0}s".format(len(data_to_send_str))
        buffer = struct.pack(fmt, self.head_code, len(data_to_send_str), data_to_send_str.encode('utf-8'))

        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            _logger.info("^^^^车辆信息发送到fms,buffer" + str(buffer))
            s.sendto(buffer, endpoint)  # 车辆信息发送到fms
        except Exception as e:
            print(e)
            _logger.error("&&&&车辆信息发送到fms,buffer is error :%s" % e)

    def get_real_ip(self):
        try:
            with open("/tmp/nodes_client_nat_ip", "r") as f:
                ip = f.read()
        except Exception as e:
            ip = 0
        return ip


# 接收节点通AP client所发送的IP地址
class NodesClientNatIPServer(object):
    def __init__(self, host='', port=9999):
        self.host = host
        self.port = port
        self.min_buffer_size = 4
        self.socket = None

    def initialize(self):
        if self.socket is None:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            try:
                self.socket.bind(('', 5010))
            except socket.error:
                _logger.error(socket.error)
                raise socket.error

    def run(self):
        try:
            self.initialize()
        except socket.error as e:
            raise socket.error
        else:
            while 1:
                try:
                    data, endpoint = self.socket.recvfrom(1024)
                    if len(data) < self.min_buffer_size:
                        continue
                    else:
                        try:
                            ip_addr = [ord(h) for h in data]
                            ip_str = ".".join([str(x) for x in ip_addr])
                            f = open("/tmp/nodes_client_nat_ip", "w")
                            f.write(ip_str)
                            f.close()
                        except Exception as e:
                            _logger.error(e)
                except KeyboardInterrupt as e:
                    _logger.error(e)
                    break


class DoublecomClientNatIp(BaseHTTPRequestHandler):
    def do_POST(self):
        form = cgi.FieldStorage(
            fp=self.rfile,
            headers=self.headers,
            environ={'REQUEST_METHOD': 'POST',
                     'CONTENT_TYPE': self.headers['Content-Type'],
                     }
        )
        ip = form.getvalue('ip')
        if os.path.exists(IP_FILE_PATH):
            os.remove(IP_FILE_PATH)
        f = open("/tmp/nodes_client_nat_ip", "w")
        f.write(ip)
        f.close()
        self.send_response(200)
        self.end_headers()
        return


def nodes_nat_ip_listener_thread(port):
    IP_FILE_PATH = "/tmp/nodes_client_nat_ip"

    if os.path.exists(IP_FILE_PATH):
        os.remove(IP_FILE_PATH)

    listener = NodesClientNatIPServer("", int(port))
    listener.run()


def upload_info_thread(server_endpoint):
    db = DBUtil()
    upload_freqency = db.get_config_value("network.ip_info_upload_freq")
    if upload_freqency == "":
        upload_freqency = 10
    upload_freqency = int(upload_freqency)
    while True:
        udp = UdpInfoServer()
        udp.send_vehicle_info(server_endpoint)

        time.sleep(upload_freqency)


def udp_info_server_thread(udp_server_port):
    listener = UdpInfoServer("", int(udp_server_port))
    listener.run()


def moxa_server_send_thread(ip):
    try:
        time.sleep(30)
        # print('start moxa server send thread')
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        PORT = 5800
        network = '<broadcast>'

        ip_bytes = ip.encode('utf-8')
        # ip_bytes = b'192.168.71.19'
        NONE_NUM = IP_BYTES_LEN - len(ip_bytes)

        # 发送数据:
        send_data = bytearray(
            b"\x00\x01\x00\x01\x00\x00\x00\x20\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xc8\x5b\x76\x9a\x44\x0a\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x1c\x00\x00\x00\x01\x00\x16")

        send_data.extend(ip_bytes)

        if NONE_NUM > 0:
            for i in range(NONE_NUM):
                send_data.append(0)
        send_data.append(0)
        send_data.extend(bytes(5800))
        send_data.append(0)
        send_data.append(0)

        _logger.info('start moxa server send thread')
        while True:
            s.sendto(send_data, (network, PORT))
            time.sleep(3)
        s.close()
    except Exception as e:
        s.close()
        _logger.info(e)
        moxa_server_send_thread(ip)


def moxa_server_recv_thread():
    # print('start moxa server recv thread')
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.bind(('', 5800))

        # 接收数据 自动阻塞 等待客户端请求:
        _logger.info('start moxa server recv thread')
        while True:
            recv_data, addr = s.recvfrom(1024)
            if len(recv_data) == 234:
                ip = addr[0]
                ip_list = ip.split('.')
                ip_list.pop(2)
                ip_list.insert(2, '88')
                ip = ".".join(str(i) for i in ip_list)
                if os.path.exists(IP_FILE_PATH):
                    os.remove(IP_FILE_PATH)
                try:
                    f = open("/tmp/nodes_client_nat_ip", "w")
                    f.write(ip)
                    f.close()
                except Exception as e:
                    print(e)
                finally:
                    if f:
                        f.close()

        s.close()
    except Exception as e:
        s.close()
        _logger.info(e)
        moxa_server_recv_thread()


def doublecom_server_thread():
    _logger.info("start doublecom nat ip server")
    sever = HTTPServer(("", 5010), DoublecomClientNatIp)
    sever.serve_forever()


def wireless_server_thread():
    ifname = "eth2"
    if os.path.exists(IP_FILE_PATH):
        os.remove(IP_FILE_PATH)
    while True:
        info = psutil.net_if_addrs()
        for k, v in info.items():
            if k != ifname:
                continue
            for item in v:
                _logger.info(item)
                if item[0] == 2 and not item[1] == '127.0.0.1':
                    try:
                        with open("/tmp/nodes_client_nat_ip", "w") as f:
                            f.write(item[1])
                    except Exception as e:
                        _logger.info(e)
                        break
            time.sleep(2)


if __name__ == '__main__':
    sros_log = sros_log.SrosLog("udp_server")
    sros_log.sendLogToFile(logging.INFO)

    db = DBUtil()

    enable_upload_ip_info = db.get_config_value("network.enable_udp_upload_ip_info")

    fms_server_ip = db.get_config_value("network.server_ip")
    fms_server_port = db.get_config_value("network.server_port")
    if fms_server_ip == "NA" or fms_server_ip == "":
        fms_server_ip = "127.0.0.1"
    if fms_server_port == "NA" or fms_server_port == "":
        fms_server_port = 8001
    fms_server_endpoint = (fms_server_ip, int(fms_server_port))

    udp_info_server_port = db.get_config_value("network.udp_server_port")
    if udp_info_server_port == "":
        udp_info_server_port = 8005

    ap = db.get_config_value("network.ap_net_mfrs_name")

    nodes_nat_ip_server_port = 5010
    ip = db.get_config_value("network.enp3s0_ip")

    t1 = threading.Thread(target=upload_info_thread, args=(fms_server_endpoint,))  # 车辆信息发送到fms
    t2 = threading.Thread(target=nodes_nat_ip_listener_thread, args=(nodes_nat_ip_server_port,))
    t3 = threading.Thread(target=udp_info_server_thread, args=(udp_info_server_port,))
    t4 = threading.Thread(target=doublecom_server_thread)
    t5 = threading.Thread(target=moxa_server_send_thread, args=(ip,))
    t6 = threading.Thread(target=moxa_server_recv_thread)
    t7 = threading.Thread(target=wireless_server_thread)
    if enable_upload_ip_info == "True":
        t1.start()
    t3.start()
    if ap == 'Nodes':
        _logger.info("start Nodes")
        t2.start()
    elif ap == 'Doublecom':
        _logger.info("start Doublecom")
        t4.start()
    elif ap == 'Moxa':
        _logger.info("Moxa")
        t5.start()
        t6.start()
    elif ap == "WirelessNetworkCard":
        _logger.info("start WirelessNetworkCard")
        t7.start()
    # t1.join()
    t3.join()
