/**
 * @file play_window.cpp
 * @author zmy (626670628@qq.com)
 * @brief 用于控制bag play的交互实现
 * @version 0.1
 * @date 2021-06-01
 * 
 * 
 */

#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <stdio.h>
#include <termio.h>
#include <unistd.h>

#include "play_window.h"

PlayWindow::PlayWindow(/* args */)
    : exit_(false),
      speed_(1.0)
{
    boost::interprocess::shared_memory_object::remove("bag_play_window_shm");
    boost::interprocess::named_mutex::remove("bag_play_window_mutex");
    boost::interprocess::named_condition::remove("bag_play_window_cnd");
}

PlayWindow::~PlayWindow()
{
    boost::interprocess::shared_memory_object::remove("bag_play_window_shm");
    boost::interprocess::named_mutex::remove("bag_play_window_mutex");
    boost::interprocess::named_condition::remove("bag_play_window_cnd");
}

int PlayWindow::scanKeyboard()
{
    int in;
    fcntl(0, F_SETFL, O_NONBLOCK);
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(STDIN_FILENO, &stored_settings); //获得stdin 输入
    new_settings = stored_settings;            //
    new_settings.c_lflag &= (~ICANON);         //
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(STDIN_FILENO, &stored_settings); //获得stdin 输入
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings); //

    in = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &stored_settings);
    return in;
}

void PlayWindow::run()
{
    std::cout << R"(
        Bag player!
    ---------------------------

      c/x : increase/decrease speeds by 10%
       z  : recover speeds to 1
    space : play/pause
       s  : single step play at pause model
       q  : quit
    )" << std::endl;

    check_exit_thread_ = std::async(std::launch::async, &PlayWindow::checkExit, this);
    boost::interprocess::managed_shared_memory managed_shm_(boost::interprocess::open_or_create, "bag_play_window_shm", 512);
    boost::interprocess::named_mutex named_mtx_(boost::interprocess::open_or_create, "bag_play_window_mutex");
    boost::interprocess::named_condition named_cnd_(boost::interprocess::open_or_create, "bag_play_window_cnd");
    while (!exit_)
    {
        int key_num = scanKeyboard();
        if (key_num == -1)
        {
            std::cout << "\r";
            std::cout << "" << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(named_mtx_);
        int *i = managed_shm_.construct<int>("play_operation")(key_num);

        if (i == 0)
            continue;
        named_cnd_.notify_all();
        named_cnd_.wait(lock);
        std::pair<float *, std::size_t> p = managed_shm_.find<float>("bag_play_speed_feedback");
        managed_shm_.destroy<float>("bag_play_speed_feedback");
        if (p.first)
        {
            speed_ = *p.first;
            std::cout << "\r";
            std::cout << " play speed:" << std::setiosflags(std::ios::fixed) << std::setprecision(1) << speed_ << std::flush;
        }
    }
}

void PlayWindow::checkExit()
{
    boost::interprocess::managed_shared_memory managed_shm_(boost::interprocess::open_or_create, "bag_play_window_shm", 512);
    boost::interprocess::named_mutex named_mtx_(boost::interprocess::open_or_create, "bag_play_window_mutex");
    boost::interprocess::named_condition named_cnd_(boost::interprocess::open_or_create, "bag_play_window_cnd");
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(named_mtx_);
    for (;;)
    {
        named_cnd_.wait(lock);
        std::pair<int *, std::size_t> p = managed_shm_.find<int>("bag_play_close_window");
        managed_shm_.destroy<int>("bag_play_close_window");
        if (p.first)
        {
            if (*p.first == -100) // 结束程序
            {
                std::cout << "recive over game!!" << std::endl;
                exit_ = true;
                break;
            }
        }
    }
}