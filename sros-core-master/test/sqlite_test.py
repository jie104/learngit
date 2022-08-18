#!/usr/bin/env python
# -*- coding:utf-8 -*-
# file sqlite_test.py
# author pengjiali
# date 19-7-3.
# copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# describe 测试sqlite3操作耗时时间
# result vc300 INSERT耗时0.00425s

import sqlite3
import time
import os

conn = sqlite3.connect('sqlite3_test.db')
c = conn.cursor()

c.execute('''CREATE TABLE IF NOT EXISTS COMPANY
       (ID INT PRIMARY KEY    NOT NULL,
       NAME           TEXT    NOT NULL,
       AGE            INT     NOT NULL,
       ADDRESS        CHAR(50),
       SALARY         REAL);''')

conn.commit()

t1 = time.time()

c.execute("INSERT INTO COMPANY (ID,NAME,AGE,ADDRESS,SALARY) \
      VALUES (1, 'Paul', 32, 'California', 20000.00 )");
conn.commit()

t2 = time.time()

print 'sqlite3 insert eslapsed time: ', (t2 - t1)

conn.close()

os.remove('sqlite3_test.db')

