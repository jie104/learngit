#!/usr/bin/python
# -*- coding: utf-8 -*-

# This shows a simple example of an MQTT subscriber.

import os
import json
import thread
import time

import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish

import sys
sys.path.append("..")
from db_util import *

ENABLE_MQTT = False
MQTT_HOST = "161.189.44.16"
MQTT_PORT = 1883
MQTT_USERNAME = "mqtt-crm"
MQTT_PASSWORD = "Sr@201820"

UPLOAD_TIME = 60 # upload state every 60s

SERIAL_ID = "NA"
with open("/sros/proc/info/serial/vehicle") as f:
    SERIAL_ID = f.read()
print("SERIAL_ID:", SERIAL_ID)

NICKNAME = "NA"
with open("/sros/proc/info/nickname") as f:
    NICKNAME = f.read()
print("NICKNAME:", NICKNAME)

TOPIC = "robot/%s"%(SERIAL_ID)

def exec_ip_route_add_cmd(host):
    cmd = "ip route add %s via 192.168.71.5" % (host)
    print(cmd)
    os.system(cmd)

def generate_ini_str(porxy_host, proxy_port, ports):
    ini = """
[common]
server_addr = %s
server_port = %d
""" % (porxy_host, proxy_port)

    for p in ports:
        # print(p)
        ps = """
[channel-%s]
type = %s
local_ip = 127.0.0.1
local_port = %d
remote_port = %d

""" % (p['name'], p['type'], p['local_port'], p['remote_port'])
        ini += ps
    print(ini)
    f = open("/sros/tool/frp/frpc.ini", "w")
    f.truncate()
    f.write(ini)
    f.close()

def do_open_channel(data):
    print(data)
    generate_ini_str(data['proxy_host'], data['proxy_port'], data['ports'])

    exec_ip_route_add_cmd(data['proxy_host'])
    os.system("systemctl restart frpc.service")

def on_connect(mqttc, obj, flags, rc):
    print("rc: " + str(rc))

def on_message(mqttc, obj, msg):
    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))

    data = json.loads(msg.payload)

    topic = msg.topic.split("/")
    if topic[0] == "robot" and topic[2] == "command":
        if data['command'] == 'CHANNEL_OPEN':
            # OPEN
            print("open channel command")

            ok = do_open_channel(data)

            d = {}
            d['command'] = "CHANNEL_OPEN_ACK"
            d['channel_id'] = data['channel_id']
            publish.single("robot/%s/command/ack"%(topic[1]), json.dumps(d), hostname=MQTT_HOST)
        if data['command'] == 'CHANNEL_CLOSE':
            # CLOSE
            print("close channel command")
            os.system("systemctl stop frpc.service")

def on_publish(mqttc, obj, mid):
    print("mid: " + str(mid))


def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))


def on_log(mqttc, obj, level, string):
    print(string)

def upload_state_thread():
    data = {}
    data['serial_id'] = SERIAL_ID
    data['nickname'] = NICKNAME
    data['state'] = {"key": "value"}

    json_str = json.dumps(data)

    while True:
        publish.single(TOPIC + "/state", json_str, hostname=MQTT_HOST)
        time.sleep(UPLOAD_TIME)

def init():

    global ENABLE_MQTT
    global MQTT_HOST
    global MQTT_PORT

    db = DBUtil()
    ENABLE_MQTT = db.get_config_value("network.enable_mqtt")
    if ENABLE_MQTT == "False" or ENABLE_MQTT == "":
        return False

    MQTT_HOST = db.get_config_value("network.mqtt_server_ip")
    MQTT_PORT = db.get_config_value("network.mqtt_server_port")
    if MQTT_HOST == "NA" or MQTT_HOST == "":
        MQTT_HOST = "161.189.44.16"
    if MQTT_PORT == "NA" or MQTT_PORT == "":
        MQTT_PORT = 1883
    print("MQTT_HOST: " + str(MQTT_HOST) +" MQTT_PORT: " + str(MQTT_PORT))

    return True

if __name__ == '__main__':

    #读取参数
    if init():
        print("enable mqtt, run mqtt...")
        exec_ip_route_add_cmd(MQTT_HOST)
        time.sleep(1) # wait for route active

        mqttc = mqtt.Client()
        mqttc.on_message = on_message
        mqttc.on_connect = on_connect
        mqttc.on_publish = on_publish
        mqttc.on_subscribe = on_subscribe
        # mqttc.username_pw_set(MQTT_USERNAME, password=MQTT_PASSWORD)
        # Uncomment to enable debug messages
        # mqttc.on_log = on_log
        mqttc.connect(MQTT_HOST, MQTT_PORT, 60)

        mqttc.subscribe(TOPIC + "/command", 0)

        # 启动状态上传线程
        thread.start_new_thread(upload_state_thread, ())
        # mqttc.publish("robot/%s/state"%(SERIAL_ID), payload=json_str, qos=0)

        mqttc.loop_forever()
    else:
        print("disable mqtt, exit...")


