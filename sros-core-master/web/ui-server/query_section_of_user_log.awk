#!/usr/bin/awk -f
# author pengjiali
# date 19-9-27.
# copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# describe 获取指定的日志：首先匹配到start_match，然后匹配match_keys，匹配query_count个数

# arguments request：
# start_match -- 首先要匹配的行，正则表达式
# query_count -- 需要匹配个数
# match_keys -- 需要匹配的keys
# invert_match_str -- 不需要匹配的

# For example: 从3.json文件中，从id为10开始，匹配10条满足：key为devie或key为sros 且没有匹配到invert_match_str 的行
#
# root@apalis-tk1:/sros/log# head 3.json -n 30
# {"id":1,"key":"sros","msg":"SROS start! Version: 4.8.0-alpha(36f334b)[develop] Sep 26 2019 08:07:34 (36f334b60b9aa4aadb139b84b16ced6f1bcac0b2)","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":2,"key":"device","msg":"Device SRC in initializing!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":3,"key":"device","msg":"Device VSC in initializing!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":4,"key":"device","msg":"Device battery in initializing!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":5,"key":"device","msg":"Device VSC communication resumes normal!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":6,"key":"device","msg":"Device battery communication resumes normal!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":7,"key":"modbus","msg":"ModbusModule: TCP start listen connection!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":8,"key":"modbus","msg":"get ip address from file: ","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":9,"key":"modbus","msg":"get ip address from adapter: 192.168.83.201","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":10,"key":"modbus","msg":"ModbusModule module start succeed!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":11,"key":"device","msg":"Device SRC communication resumes normal!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":12,"key":"device","msg":"Device MOTOR_1 in initializing!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":13,"key":"device","msg":"Device MOTOR_1 communication resumes normal!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":14,"key":"sros","msg":"SRC connected! version: 4.2.1(0000013)","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":15,"key":"device","msg":"Device MOTOR_2 in initializing!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":16,"key":"device","msg":"Device MOTOR_2 communication resumes normal!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":17,"key":"protobuf","msg":"Client 127.0.0.1:51393 connected! current client count is 1","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":18,"key":"protobuf","msg":"Client 127.0.0.1 request login!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":19,"key":"protobuf","msg":"REQUEST_LOGIN, session id is 0","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":20,"key":"protobuf","msg":"Input username is adminand password is ***","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":21,"key":"protobuf","msg":"Login authentication passed! username is admin","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":22,"key":"device","msg":"Device MOTOR_3 in initializing!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":23,"key":"device","msg":"Device MOTOR_3 communication resumes normal!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":24,"key":"protobuf","msg":"Response for login succeed!, session_id is 1569438650873","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":25,"key":"device","msg":"Device MOTOR_4 in initializing!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":26,"key":"device","msg":"Device MOTOR_4 communication resumes normal!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":27,"key":"protobuf","msg":"REQUEST_LOAD_CONFIG, session id is 1569438650873","severity":"info","time":"2019-09-26T03:10:51+08:00"}
# {"id":28,"key":"protobuf","msg":"Session 1569438650873's permission is admin","severity":"info","time":"2019-09-26T03:10:51+08:00"}
# {"id":29,"key":"protobuf","msg":"REQUEST_INFO, session id is 1569438650873","severity":"info","time":"2019-09-26T03:10:52+08:00"}
# {"id":30,"key":"sros","msg":"Last map is 20190412, last station is 0, last pose is Pose(-1.728e+38, 4.59177e-41, 0)","severity":"info","time":"2019-09-26T03:10:52+08:00"}
#
# root@apalis-tk1:/sros/log# ./query_section_of_user_log.awk -v start_match='{"id":10,' -v query_count=10 -v match_keys='"key":"device"|"key":"sros"' 3.json
# {"id":11,"key":"device","msg":"Device SRC communication resumes normal!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":12,"key":"device","msg":"Device MOTOR_1 in initializing!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":13,"key":"device","msg":"Device MOTOR_1 communication resumes normal!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":14,"key":"sros","msg":"SRC connected! version: 4.2.1(0000013)","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":15,"key":"device","msg":"Device MOTOR_2 in initializing!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":16,"key":"device","msg":"Device MOTOR_2 communication resumes normal!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":22,"key":"device","msg":"Device MOTOR_3 in initializing!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":23,"key":"device","msg":"Device MOTOR_3 communication resumes normal!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":25,"key":"device","msg":"Device MOTOR_4 in initializing!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
# {"id":26,"key":"device","msg":"Device MOTOR_4 communication resumes normal!","severity":"info","time":"2019-09-26T03:10:50+08:00"}
#

BEGIN {
    find_start=0; # 标记是否找到了开始的位置
    cur_find_count=0;  # 当前找到了多少个
#    print start_match
#    print query_count
#    print match_keys
#    print invert_match_str
}

{
    if ($0 ~ start_match)
        find_start=1;
    if (find_start) {
        if ($0 ~ match_keys && $0 !~ invert_match_str) {
            print;
            ++cur_find_count
            if (cur_find_count >= query_count) { # 获取到了指定行数可以退出了
                exit 0
            }
        }
    }
}
