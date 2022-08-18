/**
 * @file message_bag.hpp
 * @author zmy (626670628@qq.com)
 * @brief 基于boost::serialize录制和恢复数据的类
 * @version 0.1
 * @date 2021-05-18
 *
 *
 */

#ifndef MESSAGE_BAG_HPP
#define MESSAGE_BAG_HPP

#define USE_EOS

#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <shared_mutex>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <google/protobuf/message.h>

#include <boost/any.hpp>
#include <boost/filesystem.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>

#ifdef USE_EOS
#include "eos/portable_iarchive.hpp"
#include "eos/portable_oarchive.hpp"
#else
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#endif

#include "../../../serialize_patch/headers.h"
#include "../thread_pool/bag_thread_pool.h"
#include "factory/my_factory.h"

#include "core/msg/base_msg.h"
#include "core/util/time.h"
// #include <limits>

namespace bag
{

    class MsgBag : public std::enable_shared_from_this<MsgBag>
    {

    public:
        ~MsgBag() = default;
        void playBack(std::string bag_file, const bool using_terminal = false, const bool blocking = false, const bool play_with_time = true);
        void stopRecord();
        void startRecord(const std::vector<std::string> &record_msgs);
        void printAllMsgType();
        void setSpeed(const float speed);
        void setSwitchTime(const uint64_t time);
        void setPause();
        void closePlay();
        inline void setCompressBag(const bool is_compress) { compress_bag_file_ = is_compress; }
        inline void nextStep() { step_ = true; }

        static std::shared_ptr<MsgBag> Create(const std::string &bag_dir = "/sros/message_bag");

    public:
        template <typename Msg>
        auto dumpMsg(const Msg &msg, const std::string &topic);

        template <typename Msg>
        auto setMsgHandle(const std::function<void(const Msg &)> &handle, const std::string &topic = "");

    private:
        // MsgBag() = default;
        MsgBag(const std::string &bag_dir = "/sros/message_bag");
        void createBag(const uint64_t &start_time);
        auto playImp(const std::string &bag_file);
        void setMinDeltaTime(std::ifstream &bag);
        void setEndTime(std::ifstream &bag);
        void getPlayOperator();
        bool playOneMsg(std::string &msg_handler);

        template <typename Msg>
        auto playDetail(Msg &&msg, const std::string &msg_header, const uint64_t stamp);
        template <typename Msg>
        auto writeMsgHeader(const std::string &topic);
        template <typename Msg>
        auto setTime(Msg &&msg, const std::string &header, uint64_t &time);

        template <typename Msg, std::enable_if_t<!std::is_base_of<::google::protobuf::Message, Msg>::value> * = nullptr>
        auto save(const Msg &msg, const std::string &topic, const uint64_t time);

        // zmy XXX: proto是否有一个统一的基类
        template <typename Msg, std::enable_if_t<std::is_base_of<::google::protobuf::Message, Msg>::value ||
                                                 std::is_base_of<::google::protobuf::MessageLite, Msg>::value> * = nullptr>
        auto save(const Msg &msg, const std::string &topic, const uint64_t time);

        template <typename Msg>
        auto handleMsg(Msg &msg, const std::string &msg_header);

        template <typename Msg, std::enable_if_t<!std::is_base_of<::google::protobuf::Message, Msg>::value> * = nullptr>
        auto loadMsg(Msg &msg, const std::string &topic);

        template <typename Msg, std::enable_if_t<std::is_base_of<::google::protobuf::Message, Msg>::value ||
                                                 std::is_base_of<::google::protobuf::MessageLite, Msg>::value> * = nullptr>
        auto loadMsg(Msg &msg, const std::string &topic);

    private:
        std::atomic<bool> pause_;
        std::atomic<bool> playing_;
        std::atomic<bool> step_;
        std::atomic<bool> time_changed_;
        std::atomic<bool> play_with_time_;
        std::atomic<bool> compress_bag_file_;
        std::atomic<bool> using_terminal_;
        std::atomic<float> speed_;
        double time_marker_;
        int64_t begin_time_;
        uint64_t end_time_;
        uint64_t min_delta_time_;
        std::atomic<int64_t> switch_time_; /// 用于进度条控制
        int begin_cur_;                    ///数据在fsteam中的开始位置
        int end_cur_;                      /// 数据在fsteam中的结束位置
        std::string end_mark_;
        std::string bag_file_;
        std::string bag_dir_;
        std::shared_ptr<factory::Factory> factory_;
        std::unordered_map<std::string, boost::any> msg_handles_;
        std::priority_queue<uint64_t, std::vector<uint64_t>, std::greater<uint64_t>> write_cache_;
        std::vector<bool> use_msg_;
        std::vector<std::string> record_topics_;
        std::unordered_map<std::string, bool> is_compress_;
        std::ofstream record_bag_;
        std::ifstream play_bag_;
        std::mutex record_mutex_;
        std::mutex play_mutex_;
        std::condition_variable write_cv_;
        std::future<void> play_thread_;
        std::future<void> play_operator_;
        std::unique_ptr<concurrency::pool::ThreadPool> thread_pool_;
    };

} // namespace bag
#include "message_bag_impl.hpp"

#endif
