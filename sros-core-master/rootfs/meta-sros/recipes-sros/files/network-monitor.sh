#!/bin/bash
DB_FILE="/sros/db/main.db3"
function __readDB(){
    DB=$1;ITEM=$2
    result=`sqlite3 $DB "select value from config where key='$ITEM'"`
    echo $result
}

SSID_name=$(__readDB "${DB_FILE}"  "network.wifi_ssid")
password=$(__readDB "${DB_FILE}"  "network.wifi_password")
ip=$(__readDB "${DB_FILE}"  "network.wifi_ip")
gateway=$(__readDB "${DB_FILE}"  "network.wifi_gateway")
netmask_prefixlen=$(__readDB "${DB_FILE}"  "network.wifi_netmask_prefixlen")
enable_wifi=$(__readDB "${DB_FILE}"  "network.enable_wifi")

if [ -z "$SSID_name" ]
then
    # 如果wifi_ssid读取不到值，那么就试试wifi_SSID
    SSID_name=$(__readDB "${DB_FILE}"  "network.wifi_SSID")
fi

if [ -z "$SSID_name" ]
then
    # 如果wifi_ssid仍然读取不到值，那么就停止执行
    echo "SSID is empty, exit."
    exit 0
fi

if [ "$enable_wifi" != "True" ]
then
	echo "enable_wifi != True"
	exit 0
fi

#translate the string into ASCII
SSID=""
for((i=0;i<${#SSID_name};i++))
do
	tmp=`printf "%x" "'${SSID_name:$i:1}"`
	SSID="$SSID""$tmp"
done
sleep 2

#get hardware address
hwaddr_id=`ifconfig wlp1s0 | grep HWaddr | awk -F " " '{print $5}'`
hwaddr=$(echo $hwaddr_id | sed 's/://g' | tr '[A-Z]' '[a-z]')

if [ -z "$password" ]
then
	wifi_hash="wifi_""$hwaddr""_""$SSID""_managed_none"
else
	wifi_hash="wifi_""$hwaddr""_""$SSID""_managed_psk"
fi

CONNMAN_CONFIG_FILE_PATH="/var/lib/connman/""$wifi_hash"
connman_config_file="$CONNMAN_CONFIG_FILE_PATH""/settings"

connmanctl enable wifi

#write a configuration file
if [ ! -x "$CONNMAN_CONFIG_FILE_PATH" ]
then
    mkdir "$CONNMAN_CONFIG_FILE_PATH"
fi
echo "["$wifi_hash"]" > $connman_config_file
echo "Name=$SSID_name" >> $connman_config_file
echo "SSID=$SSID" >> $connman_config_file
echo "Favorite=true" >> $connman_config_file
echo "IPv4.method=manual" >> $connman_config_file
echo "IPv4.netmask_prefixlen=$netmask_prefixlen" >> $connman_config_file
echo "IPv4.local_address=$ip" >> $connman_config_file
echo "IPv4.gateway=$gateway" >> $connman_config_file
echo "Passphrase=$password" >> $connman_config_file
echo "AutoConnect=false" >> $connman_config_file

#restart wifi
connmanctl disable wifi
connmanctl enable wifi
connmanctl scan wifi

connmanctl << EOF
agent on
connect $wifi_hash
exit
EOF

#diagnose network
while true
do
    if ping -c1 -w1 "$gateway" &> /dev/null
    then
		:
    else
		echo "disconnected"
		connmanctl connect $wifi_hash
    fi
    sleep 1
done
