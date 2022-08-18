//
// Created by lfc on 17-2-22.
//

#include "keyboard_manager.h"
#include <cstring>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
// #include "glog/logging.h"

#define KEYCODE_STOP 0x05
#define KEYCODE_START 0x02
#define KEYCODE_PAUSE 0x10
#define KEYCODE_CONTINUE 0x0b
#define KEYCODE_Q 0x71
#define KEYSPACEBAR_Q 32

std::shared_ptr<KeyboardManager> KeyboardManager::record_msg_module;
KeyboardManager::KeyboardManager() {

    keyboard_thread=boost::thread(boost::bind(&KeyboardManager::monitorLoop,this));
    is_running = true;
    space_count = 0;//

    signal(SIGSEGV, KeyboardManager::systemExitTrace); //Invaild memory address
    signal(SIGABRT, KeyboardManager::systemExitTrace); // Abort signal
    signal(SIGQUIT, KeyboardManager::systemExitTrace); // Abort signal
    signal(SIGINT, KeyboardManager::systemExitTrace); // Abort signal
    signal(SIGKILL, KeyboardManager::systemExitTrace); // Abort signal
    signal(SIGSTOP, KeyboardManager::systemExitTrace); // Abort signal
}

KeyboardManager::~KeyboardManager() {
    close();
}

void KeyboardManager::monitorLoop() {
    using namespace keyboard;

    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(raw));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    //LOG(INFO)<<"Reading from keyboard\n";
    //LOG(INFO) << "---------------------------\n";
    //LOG(INFO) << "Use arrow keys to move the turtle.\n";
    char c;
    bool dirty = false;
    while (ok()) {
        // get the next event from the keyboard
        if (read(kfd, &c, 1) < 0) {
            is_running = false;
            perror("read():");
            tcsetattr(kfd, TCSANOW, &cooked);
            exit(-1);
        }
        //LOG(INFO) << "get key!%d\n" << (int)c;
        is_running = true;
        KeyCmdType cmd_type;

        switch (c) {
            case KEYSPACEBAR_Q:
                space_count++;
                if (space_count % 2 == 1) {
                    puts("PAUSE");
                    cmd_type = TYPE_CMD_PAUSE;
                    if (cmdCallback) {
                        cmdCallback(cmd_type);
                    }
                }
                if (space_count % 2 == 0) {
                    puts("CONTINUE");
                    cmd_type = TYPE_CMD_CONTINUE;
                    if (cmdCallback) {
                        cmdCallback(cmd_type);
                    }
                }
                break;
            case KEYCODE_START:
                puts("START");
                cmd_type = TYPE_CMD_START;
                if (cmdCallback) {
                    cmdCallback(cmd_type);
                }
                dirty = true;
                break;
            case KEYCODE_STOP:
                //LOG(INFO) << "STOP";
                cmd_type = TYPE_CMD_STOP;
                if (cmdCallback) {
                    cmdCallback(cmd_type);
                }
                dirty = true;
                break;
            case KEYCODE_PAUSE:
                //LOG(INFO) << "PAUSE";
                cmd_type = TYPE_CMD_PAUSE;
                if (cmdCallback) {
                    cmdCallback(cmd_type);
                }
                dirty = true;
                break;
            case KEYCODE_CONTINUE:
                puts("CONTINUE");
                cmd_type = TYPE_CMD_CONTINUE;
                if (cmdCallback) {
                    cmdCallback(cmd_type);
                }
                dirty = true;
                break;
            case KEYCODE_Q:
                is_running = false;
                puts("will quit! the keyboard will not work!");
                tcsetattr(kfd, TCSANOW, &cooked);
                return;
        }
    }
}



void KeyboardManager::join() {
    keyboard_thread.join();

}

bool KeyboardManager::ok() {
    return is_running;
}

void KeyboardManager::systemExitTrace(int signum) {
//    //LOG(INFO) << "will exit!";
    auto keyboard_module = KeyboardManager::getInstance();
    keyboard_module->close();
    abort();
}

void KeyboardManager::close() {
    is_running = false;
    tcsetattr(kfd, TCSANOW, &cooked);
}

std::shared_ptr<KeyboardManager> KeyboardManager::getInstance() {
    if (!record_msg_module) {
        record_msg_module.reset(new KeyboardManager());
    }
    return record_msg_module;
}
