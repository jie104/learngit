#!/bin/bash

# 初始化数据库文件
# 1. 建立表结构
# 2. 导入预设置记录

if [ $# -eq 0 ]; then
    DB_FILE="main.db3"
else
    DB_FILE=$1
fi

if [[ -f "${DB_FILE}" ]]; then
    rm ${DB_FILE}
fi

# 剔除文件中的空行
cat config.csv | sed '/^$/d' > /tmp/sros_db_config.csv

sqlite3 ${DB_FILE}<< EOF
.read db_schema.sql
.read db_schema_upgrade_1.sql
.read db_schema_upgrade_2.sql
.read db_schema_upgrade_3.sql
.read db_schema_upgrade_4.sql
.read db_schema_upgrade_5.sql
.read db_schema_upgrade_6.sql
.separator ","
.import /tmp/sros_db_config.csv config
EOF

function __set_meta_data() {
    KEY=$1;VALUE=$2
    r=`sqlite3 ${DB_FILE} "UPDATE meta SET value = '${VALUE}' WHERE key = '${KEY}'"`
}

# 更新db版本号
__set_meta_data "db.version" "5"

echo "=============="
echo "db file: ${DB_FILE}"
echo "list tables:"
sqlite3 ${DB_FILE}<< EOF
.tables
EOF

