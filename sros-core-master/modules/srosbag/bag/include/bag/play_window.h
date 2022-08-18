/**
 * @file play_window.h
 * @author zmy (626670628@qq.com)
 * @brief 用于控制bag play的终端
 * @version 0.1
 * @date 2021-06-01
 * 
 * 
 */

#ifndef PLAY_WINDOW_H
#define PLAY_WINDOW_H

#include <future>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_condition.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

class PlayWindow
{

public:
    PlayWindow(/* args */);
    ~PlayWindow();
    PlayWindow(const PlayWindow &) = delete;
    PlayWindow &operator=(const PlayWindow &) = delete;
    void run();

private:
    int scanKeyboard();
    void checkExit();

private:
    bool exit_;
    float speed_;
    std::future<void> check_exit_thread_;
};

#endif