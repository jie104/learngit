#!/bin/bash

gpioset 0 30=0
sleep 1
gpioset 0 30=1

#等待8S 重启完毕