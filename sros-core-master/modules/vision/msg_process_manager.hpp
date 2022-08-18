//
// Created by lfc on 18-4-18.
//

#ifndef _MSG_PROCESS_MANAGER_HPP
#define _MSG_PROCESS_MANAGER_HPP

#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <glog/logging.h>

namespace msg_process{
template<typename ThreadInputPara_Ptr>
class MsgProcessManager {
public:
    MsgProcessManager(const boost::function<void(ThreadInputPara_Ptr)> &func,int queue_size = 1) :queue_size_thresh(queue_size){
        setMatchCallback(func);
        reset();
    }

    virtual ~MsgProcessManager() {
        reset();
    }

    void setMatchCallback(const boost::function<void(ThreadInputPara_Ptr)> &func) {
        matchCallback = func;
    }

    void close() {
        reset();
    }

    void creatMatchThread() {
        close();
        running_state = true;
        setWakeupState();
        thread_manager.reset(new boost::thread(boost::bind(&MsgProcessManager::handleMatchThread, this)));
    }

    void pushToQueue(const ThreadInputPara_Ptr &input) {
        if (ok()) {
            pushQueue(input);
            if (getWakeupState()) {
                condition.notify_all();
            }
        }else {
            LOG(INFO) << "have not set the running state!";
        }
    }

    bool isEmpty() {
        if (input_queue.size() == 0) {
            return true;
        }
        return false;
    }

    bool ok() {
        return running_state;
    }


    ThreadInputPara_Ptr popQueue(){
        ThreadInputPara_Ptr match_input_msg;
        boost::mutex::scoped_lock lock(stack_mutex);
        if (!input_queue.empty()) {
            match_input_msg = input_queue.front();//从前头取出
            input_queue.pop_front();
        }
        return match_input_msg;
    }

    void clear(){
        boost::mutex::scoped_lock lock(stack_mutex);
        input_queue.clear();
    }


    int queueSize() {
        return input_queue.size();
    }


private:

    void reset() {
        running_state = false;
        condition.notify_all();
        if (thread_manager) {
            thread_manager->join();
            thread_manager.reset();
        }
    }

    void handleMatchThread() {
        while (ok()) {
            if (input_queue.empty()) {
                boost::unique_lock<boost::mutex> lock(condition_mutex);
                boost::xtime xt;
#if BOOST_VERSION >= 105000
                boost::xtime_get(&xt, boost::TIME_UTC_);
#else
                boost::xtime_get(&xt, boost::TIME_UTC);
#endif
                xt.nsec += 5e8;
                setWakeupState();
                if(condition.timed_wait(lock,xt)) {
                    resetWakeupState();
                    if (!ok()) {
                        continue;
                    }
                }else {
                    if (!ok()) {
                        continue;
                    }
                    if (!input_queue.empty()) {
                        LOG(INFO) << "err to wake up the thread!";
                        continue;
                    }else {
//                        LOG(INFO) << "err to get the scan! wait time too long!";
                        continue;
                    }
                }
            }
            resetWakeupState();
            ThreadInputPara_Ptr input_para = popQueue();
            if (!input_para) {
                LOG(INFO) << "!!!!err to get the input para! maybe the para has been deleted!";
                continue;
            }
            if (matchCallback) {
                matchCallback(input_para);
            }else {
                LOG(INFO) << "err to get the callback!";
            }

        }
    }

    void setWakeupState() {
        need_processor_wakeup = true;
    }

    void resetWakeupState() {
        need_processor_wakeup = false;
    }

    bool getWakeupState() {
        return need_processor_wakeup;
    }

    void pushQueue(const ThreadInputPara_Ptr &input_para){
        boost::mutex::scoped_lock lock(stack_mutex);
        if (queueSize() <= queue_size_thresh) {
            input_queue.push_back(input_para);//push到结尾
        }
    }

private:
    boost::function<void(ThreadInputPara_Ptr)> matchCallback;
    bool need_processor_wakeup;
    bool running_state;

    int queue_size_thresh;
    std::deque<ThreadInputPara_Ptr> input_queue;
    std::shared_ptr<boost::thread> thread_manager;
    boost::mutex condition_mutex;
    boost::condition_variable_any condition;
    boost::mutex stack_mutex;
};

}


#endif //SROS_THREAD_MANAGER_HPP
