#!/bin/bash

cd ../tools/monitor_file_to_json
cp ../../core/proto/monitor.proto ./temp.proto
sed -e '/optimize_for/d'  ./temp.proto >./monitor_protobuf_full.proto
protoc --cpp_out=. --proto_path=./  ./monitor_protobuf_full.proto
rm ./temp.proto
cd -

