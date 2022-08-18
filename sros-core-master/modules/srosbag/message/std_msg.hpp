/**
 * @file std_msgs.h
 * @author zmy (626670628@qq.com)
 * @brief 一些通用msgs
 * @version 0.1
 * @date 2021-03-30
 * 
 * 
 */

#ifndef STD_MSGS_H
#define STD_MSGS_H

#include <string>

namespace std_msgs
{
    struct Header
    {
        uint64_t stamp;      //us
        uint64_t sync_stamp; // us
        std::string frame_id;
    };
}

#endif