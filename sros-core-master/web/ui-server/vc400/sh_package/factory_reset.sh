#!/bin/bash

#需要知道gpio恢复出厂设置需要多久生效
gpioset 3 26=0
sleep 8
gpioset 3 26=1

