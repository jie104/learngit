# -*- coding:utf-8 -*-
# file generate_maps_db.py
# author YangHuaiDong
# date 2021/11/17 下午3:31
# copyright Copyright (c) 2021 Standard Robots Co., Ltd. All rights reserved.
# describe

from utils import (
    SROS_MAP_DIR,
    get_map_name_list,
    MapsAdmin,
    get_map_item_info
)


def generate_maps_db_data():
    map_name_file_list = get_map_name_list(map_dir=SROS_MAP_DIR)
    map_name_file_display_list = []
    for map_name in map_name_file_list:
        map_name_file_display_list.append(map_name.decode('utf-8'))

    map_name_table_list = []
    map_name_table_list_resp = MapsAdmin.get_all_maps()

    for map_info in map_name_table_list_resp:
        map_name_table_list.append(map_info["name"])

    map_name_file_display_list_set = set(map_name_file_display_list)
    map_name_table_list_set = set(map_name_table_list)

    add_map_name_list = list(map_name_file_display_list_set - map_name_table_list_set)
    up_map_name_list = list(map_name_file_display_list_set & map_name_table_list_set)
    rm_map_name_list = list(map_name_table_list_set - map_name_file_display_list_set)

    for name in rm_map_name_list:
        MapsAdmin.delete_map_by_name(name)

    for name in add_map_name_list:
        map_info = get_map_item_info(map_name=name.encode('utf-8'), map_dir=SROS_MAP_DIR)
        map_info['name'] = map_info['name'].decode('utf-8')
        MapsAdmin.create_map(map_info)

    for name in up_map_name_list:
        map_table_item = MapsAdmin.get_map_info_by_name(name)
        map_info = get_map_item_info(map_name=name.encode('utf-8'), map_dir=SROS_MAP_DIR)

        if map_info['md5'] != map_table_item['md5']:
            map_info['name'] = map_info['name'].decode('utf-8')
            MapsAdmin.update_map_by_name(name, map_info)


if __name__ == '__main__':
    generate_maps_db_data()
