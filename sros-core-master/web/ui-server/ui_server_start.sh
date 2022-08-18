#!/usr/bin/env bash

DB_FILE="/sros/db/main.db3"
function __readDB(){
    DB=$1;ITEM=$2
    result=`sqlite3 $DB "select value from config where key='$ITEM'"`
    echo $result
}

#设置系统环境变量
export LANG=en_US.UTF-8

http_port=$(__readDB "${DB_FILE}"  "network.http_service_port")

/usr/bin/python /sros/web/ui-server/code.py ${http_port}

echo "ui service start"
