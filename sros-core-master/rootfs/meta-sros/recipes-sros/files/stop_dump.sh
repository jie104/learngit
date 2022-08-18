#!/bin/bash

echo "enter $0"

ps_cmd=`ps -aux | grep "$1" | awk '{print $2}'`
kill_cmd="kill -9"
for pid in $ps_cmd; do
  kill_cmd=${kill_cmd}" $pid";
  break;
done

echo $kill_cmd
$($kill_cmd)

echo "exit $0"
