#!/usr/bin/env bash


DB_FILE="/sros/db/main.db3"
OLD_VERSION="4.17.0"

function __last_time() {
  DB=$1
  last_alive_time=`sqlite3 $DB "select last_alive_time from run_log order by last_alive_time DESC limit 1"`
  echo $last_alive_time
}

function __checkMaps() {
  DB=$1;last_alive_time=$2
  new_version=`sqlite3 $DB "select sros_version from run_log where last_alive_time='$last_alive_time'"`
  echo $new_version
}

last_alive_time=$(__last_time "${DB_FILE}")
new_version=$(__checkMaps "${DB_FILE}"  "$last_alive_time")

split_version=${new_version: 0: 6}
if [[ "$split_version" < "$OLD_VERSION" ]];then
  /usr/bin/python /sros/web/ui-server/generate_maps_db.py &
  echo "maps update successful!"
else
  echo "maps not update!"
fi