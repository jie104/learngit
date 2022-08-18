//
// Created by jintongxing on 2020/3/9.
//
#ifndef SDKELI_LSPDM_UDP_KELI_DRIVER_H
#define SDKELI_LSPDM_UDP_KELI_DRIVER_H
#include "sdkeli_ls_common_udp.hpp"
#include <glog/logging.h>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <thread>

namespace laser {
    template <class ScanMsg>
    class KeliDriver{
    public:
        using ScanMsg_ptr = std::shared_ptr<ScanMsg>;
        /*配置雷达IP、端口号
         * 输入：
         * str_host_name: IP地址,192.168.1.100
         * str_port: 2112端口号*/
        KeliDriver(const std::string& str_host_name = "192.168.1.100", const std::string& str_port = "2112"):str_host_name_(str_host_name), str_port_(str_port),is_ok_(false){
            parser_ = new sdkeli_lspdm_udp::CSDKeliLs1207DEParser;
            time_limit_ = 5;
            common_udp_ = NULL;
            LOG(INFO) << "Initialized KeliDriver!";
        }

        ~KeliDriver(){//main函数结束的时候自动调用
            if(common_udp_ != NULL){//向设备发送指令关闭通信，停止socket
                if(common_udp_->isConnected()){//防止没有调用close就析构了driver
                    common_udp_->setConnectFlag(false);
                    if(read_data_thread_.joinable()){
                        read_data_thread_.join();
                        LOG(INFO) << "Stop thread while destructing the driver!";
                    }
                }
                delete common_udp_;//这里的析构实际上也完成了close的工作
                common_udp_ = nullptr;
            }
            if(parser_ != NULL){
                delete parser_;
                parser_ = nullptr;
            }
        }

        /*建立socket连接，发送指令开始接收数据*/
        bool open(){
            if(common_udp_ != NULL){
                delete common_udp_;
                common_udp_ = nullptr;
            }
            common_udp_ = new sdkeli_lspdm_udp::CSDKeliLsCommonUdp(str_host_name_, str_port_, time_limit_, parser_);//使用基指针指向继承类，这里主要是ros发布消息相关的初始化
            bool result = common_udp_->Init();//初始化，包括socket相关，以及像设备发送开始接受stream的指令
            if(result){
                LOG(INFO) << "Open successfully!";
                is_ok_ = true;
            }else{
                is_ok_ = false;
                LOG(INFO) << "Failed to open!";
            }
            return result;
        }
        /*创建新线程不断主动获取数据*/
        bool start(){
            if(common_udp_->isConnected()){
                read_data_thread_ = std::thread(std::bind(&KeliDriver::getScanData, this));//打开成功，创建新线程不断获取新数据
                LOG(INFO) << "New thread has been built!";
                return true;
            }else{
                LOG(INFO) << "Disconnected while start!";
                return false;
            }
        }

        /*关闭获取数据的线程，发送指令停止传输，断开socket连接*/
        void close(){
            LOG(INFO) << "Closing...";
            if (common_udp_) {
                common_udp_->setConnectFlag(false);//修改标志位，结束接收数据的线程
                if(read_data_thread_.joinable()){
                    read_data_thread_.join();//等待线程结束，再往下执行，不然可能提前断开了socket连接
                }
                LOG(INFO) << "Thread has been joined!";
                common_udp_->StopScanner();//向设备发送指令
                common_udp_->CloseDevice();//关闭socket
            }
        }

        const bool ok() const{ return is_ok_; }

        /*重新初始化common_udp_，重新建立连接和接收数据的线程*/
        void reboot(){
            LOG(INFO) << "Rebooting";
            if(common_udp_){
                close();
                open();
            }
        }

        /*从队列中获取单帧激光雷达数据，
         * return：成功返回true;最多等待1s，返回false*/
        bool getScan(ScanMsg_ptr& scan){//从queue中获取
            if(scans_queue_.empty()){//如果全部被取走了，等待被唤醒
                std::unique_lock<std::mutex> lock(condition_mutex_);
                auto state = condition_variable_.wait_for(lock, std::chrono::seconds(1));//直到被唤醒或者超时，超时的原因包括从雷达获取scan的线程由于错误终止了，需要重启
                if(state == std::cv_status::timeout){
                    LOG(INFO) << "Time out for get scan as queue is empty!";
//                    is_ok_ = false;
                }
            }
            if(!scans_queue_.empty()){
                scan = popScan();//reset
//                LOG(INFO)<<"scan info:"<<scan->scan_time<<","<<scan->angle_min<<","<<scan->angle_max;
            }else{
                LOG(INFO) << "Cannot get scan data";
                return  false;
            }
            return true;
        }

    private:
        sdkeli_lspdm_udp::CSDKeliLs1207DEParser* parser_;
        sdkeli_lspdm_udp::CSDKeliLsCommon* common_udp_;
//        sdkeli_lspdm_udp::LidarConfig config_;
        std::string str_host_name_;
        std::string str_port_;
        int time_limit_;

        std::mutex condition_mutex_;
        std::mutex read_and_write_mutex_;
        std::condition_variable_any condition_variable_;
        std::queue<ScanMsg_ptr> scans_queue_;
        std::thread read_data_thread_;
        bool is_ok_;
        const int get_scan_try_count_ = 8;

        /*将获取的完整的scan push到queue中。最多保存5帧，否则清除旧数据*/
        void pushScan(const ScanMsg_ptr& scan){
            read_and_write_mutex_.lock();
            scans_queue_.push(scan);
            if(scans_queue_.size() > 2){
                LOG(INFO) << "Too many scans stored: " << scans_queue_.size();
            }
            if(scans_queue_.size() > 5){//如果push后大于5个，被清理到只剩2个
                while(scans_queue_.size() > 2){
                    scans_queue_.pop();
                }
            }
//            LOG(INFO) << "The size of queue: " << scans_queue_.size();
            read_and_write_mutex_.unlock();
        }

        /*单独的线程，不断从雷达主动获取scan
         * 返回：若无法从雷达获得报文，或者连续多次无法获得完整的scan，返回false结束线程；否则，死循环不断获取新数据。
         * */
        bool getScanData(){
            int result = sdkeli_lspdm_udp::ExitSuccess;
            int invalid_scan_count = 0;//
            while(common_udp_->isConnected()){
                if(invalid_scan_count < get_scan_try_count_){//正常情况下，4次循环一次完整的scan ,设置无效循环的上限
                    ScanMsg_ptr scan(new ScanMsg);
                    scan->time_ = sros::core::util::get_time_in_us();
                    result = common_udp_->LoopOnce(*scan);
                    if(result == sdkeli_lspdm_udp::ExitSuccess){
                        if(!scan->ranges.empty()){
                            pushScan(scan);
                            condition_variable_.notify_all();//一定要唤醒，不然被自己取空后，一直在等待，直到超时退出，重新获取
                            invalid_scan_count = 0;
                        }else{
                            invalid_scan_count++;
                        }
                    }else{
                        is_ok_ = false;
                        LOG(ERROR) << "Error while trying to get scan data from lidar!";
                        return false;
                    }
                }else{
                    is_ok_ = false;
                    LOG(ERROR) << "Time out while getting a whole frame!";
                    return false;
                }
            }
            LOG(INFO) << "Disconnected while getting data from lidar";
            is_ok_ = false;
            return false;
        }

        /*从队列里获取最新的一帧scan*/
        ScanMsg_ptr popScan(){
            ScanMsg_ptr scan;
            read_and_write_mutex_.lock();
            if(!scans_queue_.empty()){
                scan = scans_queue_.front();
                scans_queue_.pop();//保证不能为空
            }
            read_and_write_mutex_.unlock();
            return scan;
        }
    };
}

#endif //SDKELI_LSPDM_UDP_KELI_DRIVER_H
