//
// Created by lfc on 17-2-22.
//

#ifndef GPS_RECEIVER_KEYBOARD_MANAGER_H
#define GPS_RECEIVER_KEYBOARD_MANAGER_H
#include <termios.h>
#include <signal.h>
#include <boost/function.hpp>
#include <boost/thread.hpp>
namespace keyboard{
    enum KeyCmdType{
        TYPE_CMD_PAUSE = 1,
        TYPE_CMD_CONTINUE = 2,
        TYPE_CMD_START = 3,
        TYPE_CMD_STOP = 4,
    };
}

typedef boost::function<void(keyboard::KeyCmdType)> keyboardCallbackFunc;
class KeyboardManager {
public:

    virtual ~KeyboardManager();
    void monitorLoop();

    void join();

    bool ok();

    void close();

    static std::shared_ptr<KeyboardManager> getInstance();

    void setCmdCallback(keyboardCallbackFunc func_) {
        cmdCallback = func_;
    }

private:
    keyboardCallbackFunc cmdCallback;

    KeyboardManager();

    static void systemExitTrace(int signum);

    static std::shared_ptr<KeyboardManager> record_msg_module;

    int kfd = 0;
    struct termios cooked, raw;
    boost::thread keyboard_thread;
    bool is_running;
    int space_count;

};



#endif //GPS_RECEIVER_KEYBOARD_MANAGER_H
