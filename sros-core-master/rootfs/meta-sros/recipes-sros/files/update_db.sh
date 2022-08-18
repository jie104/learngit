#!/usr/bin/env bash

DB_FILE=/sros/db/main.db3

result=`sqlite3 ${DB_FILE} "SELECT value FROM config WHERE key = '$1'"`;
echo "Before: $1 => ${result}"

if [ "$2" = "" ]; then
  # 不进行更新操作
  exit 0
fi

result=`sqlite3 ${DB_FILE} "UPDATE config SET value = '$2' WHERE key = '$1'"`;

result=`sqlite3 ${DB_FILE} "SELECT value FROM config WHERE key = '$1'"`;
echo "After: $1 => ${result}"
