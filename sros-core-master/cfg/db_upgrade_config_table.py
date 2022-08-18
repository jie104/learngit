#!/usr/bin/env python
# -*- coding:utf-8 -*-
# Author: pengjiali
# describe: 更新config表：
# 1. 添加新配置
# 2. 将被删除的配置设置为删除
# 3. 更新除用户设置的值以外的所有字段

import sqlite3
import argparse
import csv
import os

parser = argparse.ArgumentParser(usage='更新config表', description='开始更细config表：')
parser.add_argument('db_file_path', type=str, help='数据库的路径')
parser.add_argument('csv_file_path', type=str, help='csv的路径')
args = parser.parse_args()

print args
db_file_path = args.db_file_path
csv_file_path = args.csv_file_path

# db_file_path = '/sros/db/main.db3'
# csv_file_path = '/sros/update/config.csv'

if not os.path.exists(db_file_path):
    print '数据库的路径不存在！'
    exit(-1)

if not os.path.exists(csv_file_path):
    print 'csv的路径不存在！'
    exit(-1)

conn = sqlite3.connect(db_file_path)
conn.text_factory = str
cur = conn.cursor()
cur.execute('SELECT id FROM config')

db_table = []  # config 表
for row in cur:
    db_table.append(row)

csv_table = []  # csv 表
with open(csv_file_path, 'rb') as csvfile:
    fieldnames = ['id', 'key', 'name', 'value', 'default_value', 'value_units', 'value_type',
                  'value_range', 'description', 'permission', 'changed_time', 'changed_user', 'is_valid']
    reader = csv.DictReader(csvfile, fieldnames=fieldnames)
    for csv_row in reader:
        csv_row['id'] = int(csv_row['id'])  # csv读进来的都是str，此处将id转换成int类型
        csv_table.append(csv_row)

# 第一遍遍历，将csv中的新的行插入数据库，将csv中被修改的行同步到数据库
for csv_row in csv_table:
    is_db_row_exist = False  # 查找数据库config表，看csv_row当前行是否已经在数据库config表中存在了
    for db_row in db_table:
        if csv_row['id'] == db_row[0]:
            is_db_row_exist = True
            break

    if is_db_row_exist:  # 若csv_row当前行已经在数据库config表中存在了就更新数据库中除value之外的所有字段
        cur.execute("UPDATE config SET key=:key, name=:name, default_value=:default_value,"
                    " value_units=:value_units, value_type=:value_type, value_range=:value_range,"
                    " description=:description, permission=:permission, changed_time=:changed_time, "
                    "changed_user=:changed_user, is_valid=:is_valid WHERE id=:id",
                    {"key": csv_row['key'], "name": csv_row['name'],
                     "default_value": csv_row['default_value'],
                     "value_units": csv_row['value_units'],
                     "value_type": csv_row['value_type'],
                     "value_range": csv_row['value_range'],
                     "description": csv_row['description'],
                     "permission": csv_row['permission'],
                     "changed_time": csv_row['changed_time'],
                     "changed_user": csv_row['changed_user'],
                     "is_valid": csv_row['is_valid'],
                     "id": csv_row['id']})
    else:  # 若csv_row中的当前行数据库config表中不存在就插入
        cur.execute("INSERT INTO config VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                    (csv_row['id'], csv_row['key'], csv_row['name'], csv_row['value'],
                     csv_row['default_value'], csv_row['value_units'],
                     csv_row['value_type'], csv_row['value_range'],
                     csv_row['description'], csv_row['permission'],
                     csv_row['changed_time'], csv_row['changed_user'],
                     csv_row['is_valid']))

# 第二次遍历， 数据库中多余的项删除掉
for db_row in db_table:
    is_csv_row_exist = False  # 标记数据库中的当前行是否已经在csv中存在了
    for csv_row in csv_table:
        if csv_row['id'] == db_row[0]:
            is_csv_row_exist = True
            break

    if not is_csv_row_exist:
        cur.execute("UPDATE config SET is_valid=0 WHERE id=:id", {"id": db_row[0]})

conn.commit()
conn.close()

print '完成更新config表！'
