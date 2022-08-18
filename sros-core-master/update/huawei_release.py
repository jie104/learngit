#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @File: huawei_release.py
# @Author: pengjiali
# @Date: 20-3-6
# @Copyright: Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# @Describe: 给华为发布的版本需要将压缩包拆分成小于14M的部分挨个发送


import smtplib
from email.mime.text import MIMEText
from email.mime.application import MIMEApplication
from email.header import Header
from email.mime.multipart import MIMEMultipart
import argparse
import shutil
import os

parser = argparse.ArgumentParser(usage='华为邮件发布', description='给华为发布的版本需要将压缩包拆分成小于14M的部分挨个发送邮件：')
parser.add_argument('mail_title', type=str, help='邮件主题')
parser.add_argument('source_file', type=str, help='目标文件')
parser.add_argument('mail_user', type=str, help='邮件用户名')
parser.add_argument('mail_password', type=str, help='邮件密码')
args = parser.parse_args()

dst_path = "/tmp/huawei_release/"
if os.path.exists(dst_path):
    shutil.rmtree(dst_path)
os.makedirs(dst_path)

source_file = args.source_file
cmd = "zip -r -s 13m " + dst_path + source_file + ".zip " + source_file
print(cmd)
val = os.system(cmd)
if val != 0:
    raise Exception("uzip failed!")
else:
    print('zip succeed!')

# 第三方 SMTP 服务
mail_host = "smtphz.qiye.163.com"  # 设置服务器
mail_user = args.mail_user  # 用户名
mail_pass = args.mail_password  # 口令

sender = mail_user
receivers = ['chenbaobing@standard-robots.com', 'jianghongyuan@standard-robots.com', 'hongxiang.li@standard-robots.com',
             'chenyuxin@standard-robots.com', 'huangjianfeng@standard-robots.com', 'zengdekang@standard-robots.com',
             'zhangzhixin@standard-robots.com', 'zhangwenjie@standard-robots.com']
receivers_str = ""
for receiver in receivers:
    receivers_str += receiver
    receivers_str += ','
receivers_str = receivers_str[:-1]

smtpObj = smtplib.SMTP()
smtpObj.connect(mail_host, 25)  # 25 为 SMTP 端口号
smtpObj.login(mail_user, mail_pass)


def sendMail(test, attach_file):
    global smtpObj, sender, args
    # 邮件正文内容
    message = MIMEMultipart()
    message['From'] = Header(sender, 'utf-8')
    message['To'] = Header(receivers_str, 'utf-8')

    subject = args.mail_title
    message['Subject'] = Header(subject, 'utf-8')
    message.attach(MIMEText(test, 'plain', 'utf-8'))

    zipApart = MIMEApplication(open(dst_path + attach_file, 'rb').read())
    zipApart.add_header('Content-Disposition', 'attachment', filename=attach_file)

    message.attach(zipApart)

    smtpObj.sendmail(sender, receivers, message.as_string())


sendMail("part 0", source_file + ".zip")
sendMail("part 1", source_file + ".z01")
sendMail("part 2", source_file + ".z02")

print("send mail succeed!")
