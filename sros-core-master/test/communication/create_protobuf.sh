#!/usr/bin/env bash
# @describe： 生成protobuf的脚本


sed -ie 's/import "core\/proto\/monitor.proto/import "monitor.proto/g' ../../core/proto/main.proto

protoc --python_out=. --proto_path=../../core/proto/ ../../core/proto/monitor.proto
protoc --python_out=. --proto_path=../../core/proto/ ../../core/proto/main.proto

sed -ie 's/import "monitor.proto/import "core\/proto\/monitor.proto/g' ../../core/proto/main.proto