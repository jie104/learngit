#!/bin/bash

echo "enter $0"

func_name=$1
if [[ $func_name == 'ETH0' ]]; then
    tcpdump -i any host $3 -w $2$func_name.data
elif [[ $func_name == 'ENP3S0' ]]; then
    tcpdump -i any host $3 -w $2$func_name.data
elif [[ $func_name == 'CAN' ]]; then
    candump > $2$func_name.data
fi

echo "exit $0"