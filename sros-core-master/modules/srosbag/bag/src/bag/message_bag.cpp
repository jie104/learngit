/**
 * @file message_bag.cpp
 * @author zmy (626670628@qq.com)
 * @brief 基于boost::serialize录制和恢复数据的类实现
 * @version 0.1
 * @date 2021-05-19
 *
 *
 */

#include <chrono>
#include <cstdlib>
#include <glog/logging.h>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdlib.h>

#include "boost/date_time/posix_time/posix_time_types.hpp"
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_condition.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "bag/message_bag.h"
#include "play_window/config.h"

#ifdef USE_EOS
namespace boost
{
    namespace archive
    {

        // explicitly instantiate for this type of binary stream
        template class basic_binary_oarchive<eos::portable_oarchive>;
        template class basic_binary_oprimitive<
            eos::portable_oarchive
#if BOOST_VERSION < 103400
            ,
            std::ostream
#else
            ,
            std::ostream::char_type, std::ostream::traits_type
#endif
            >;

        template class basic_binary_iarchive<eos::portable_iarchive>;

        template class basic_binary_iprimitive<
            eos::portable_iarchive
#if BOOST_VERSION < 103400
            ,
            std::istream
#else
            ,
            std::istream::char_type, std::istream::traits_type
#endif
            >;

    }
} // namespace boost::archive

#endif

namespace bag
{
    std::vector<std::string> splitStr(const std::string &str, const char delim)
    {
        std::vector<std::string> splited;
        std::stringstream sstr(str);
        std::string splited_str;
        while (getline(sstr, splited_str, delim))
        {
            splited.emplace_back(splited_str);
        }
        return splited;
    }

    std::shared_ptr<MsgBag> MsgBag::Create(const std::string &bag_dir)
    {
        static auto bag = std::shared_ptr<MsgBag>(new MsgBag(bag_dir));
        return bag;
    }

    MsgBag::MsgBag(const std::string &bag_dir)
        : playing_(false),
          pause_(false),
          time_changed_(false),
          step_(false),
          play_with_time_(true),
          compress_bag_file_(false),
          speed_(1.0),
          time_marker_(0),
          begin_time_(0),
          end_time_(0),
          min_delta_time_(100000),
          switch_time_(-1),
          begin_cur_(0),
          end_cur_(0),
          bag_file_(""),
          bag_dir_(bag_dir),
          factory_(factory::getFactory()),
          end_mark_("ENDMESSAGE"),
          use_msg_(std::vector<bool>(Container::types_size, false)),
          thread_pool_(nullptr)
    {
        printAllMsgType();
        if (!(boost::filesystem::exists(boost::filesystem::path(bag_dir_)) &&
              boost::filesystem::is_directory(boost::filesystem::path(bag_dir_))))
        {
            LOG(WARNING) << "without message bag directory,create directory:" << bag_dir_;
            boost::filesystem::create_directories(boost::filesystem::path(bag_dir_));
        }
        LOG(INFO) << "successfully to create MsgBag Instance";
    }

    void MsgBag::stopRecord()
    {
        use_msg_ = std::vector<bool>(Container::types_size, false);
        record_topics_.clear();
        if (thread_pool_ != nullptr)
        {
            thread_pool_->stop();
            thread_pool_.reset();
        }

        if (record_bag_.is_open())
        {
            record_bag_.close();
            if (bag_file_ != "")
            {
                if (compress_bag_file_)
                {
                    auto path = boost::filesystem::path(bag_file_);
                    auto dir = path.parent_path() / path.stem();
                    boost::filesystem::create_directories(boost::filesystem::path(dir));
                    boost::filesystem::rename(path, dir / path.filename());

                    // std::string cmd = "cd " + bag_dir_ + " && tar -Jcvf " +
                    //                   path.stem().generic_string() + ".tar.xz " + path.stem().generic_string();
                    std::string cmd = "cd " + bag_dir_ + " && tar -zcvf " +
                                      path.stem().generic_string() + ".tar.gz " + path.stem().generic_string();
                    auto sys = std::system(cmd.c_str());
                    boost::filesystem::remove_all(dir);
                }

                bag_file_ = "";
            }
        }
        // LOG(INFO) << "Stop to record message!";
    }

    void MsgBag::startRecord(const std::vector<std::string> &record_msgs)
    {

        std::lock_guard<std::mutex> lock(record_mutex_);
        thread_pool_.reset(new concurrency::pool::ThreadPool(3));
        for (const auto &msg_name : record_msgs)
        {
            auto i = factory_->getClassID(msg_name);
            if (i != -1)
            {
                use_msg_[i] = true;
                record_topics_.emplace_back(msg_name);
            }
            else
            {
                LOG(INFO) << msg_name << " is not registered,please register it!!!!";
            }
        }
    }

    void MsgBag::createBag(const uint64_t &start_time)
    {
        auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::stringstream time_ss;
        time_ss << std::put_time(std::localtime(&time), "%F-%H-%M-%S");
        bag_file_ = bag_dir_ + "/" + time_ss.str() + ".sbag";
        LOG(INFO) << "ready record message to " << bag_file_;
#ifdef USE_EOS
        record_bag_.open(bag_file_, std::ios_base::out | std::ios_base::binary);
#else
        record_bag_.open(bag_file_, std::ios_base::out);
#endif
        if (record_bag_.is_open())
        {
            LOG(INFO) << "successfuly create message bag " << bag_file_;
            record_bag_ << start_time << ";";

            LOG(INFO) << "The following message will be recorded:";
            for (auto &topic : record_topics_)
            {
                auto i = factory_->getClassID(topic);
                if (i == -1)
                    continue;
                auto msg = Container::createMsg(i);
                boost::apply_visitor([this, &topic](auto &&msg)
                                     { this->writeMsgHeader<std::decay_t<decltype(msg)>>(topic); },
                                     msg);
            }
            record_bag_ << std::endl;
        }
        else
        {
            LOG(WARNING) << "failed to create message bag  " << bag_file_;
        }
    }

    auto MsgBag::playImp(const std::string &bag_file)
    {
#ifdef USE_EOS
        play_bag_.open(bag_file, std::ios_base::in | std::ios_base::binary);
#else
        play_bag_.open(bag_file, std::ios_base::in);
#endif

        if (play_bag_.is_open())
        {
            std::string bag_info;
            getline(play_bag_, bag_info);
            auto split_str = splitStr(bag_info, ';');
            if (!split_str.empty())
            {
                begin_time_ = static_cast<uint64_t>(std::stoll(split_str[0]));
                LOG(INFO) << "successfully open bag file with info:\n";

                std::vector<std::string> miss_name;
                for (size_t i = 1; i < split_str.size(); ++i)
                {
                    auto info = splitStr(split_str[i], ',');
                    if (factory_->getClassID(info[0]) == -1)
                    {
                        miss_name.emplace_back(split_str[i]);
                        continue;
                    }

                    is_compress_[info[0]] = std::stoi(info[1]);

                    LOG(INFO) << split_str[i] << "\n";
                }
                if (!miss_name.empty())
                {
                    LOG(INFO) << "The follow types is missing:\n";
                    for (const auto &str : miss_name)
                    {
                        LOG(INFO) << str << "\n";
                    }
                }

                setEndTime(play_bag_);
                setMinDeltaTime(play_bag_);

                if (end_time_ < begin_time_)
                {
                    LOG(ERROR) << "The bag end time is smaller than begin time!!! please check it!!";
                    return;
                }
                if (using_terminal_)
                {
                    std::string cmd = std::string("gnome-terminal --working-directory=") + PLAY_WINDOW_EXE + " -- ./terminal_play";
                    auto sys = std::system(cmd.c_str());
                    play_operator_ = std::async(std::launch::async, &MsgBag::getPlayOperator, this);
                }

                time_marker_ = sros::core::util::get_timestamp_in_us();
            }
            else
            {
                LOG(WARNING) << "The bag info is beak, please check it!!";
            }
        }
        else
        {
            LOG(INFO) << "failed to open bag file, please to check bag file";
            return;
        }

        std::string msg_header;
        while (playing_)
        {
            if (pause_)
            {
                time_changed_ = true;
                if (step_)
                {
                    step_ = false;
                    if (!playOneMsg(msg_header))
                        break;
                }
                std::this_thread::yield();
                continue;
            }
            if (!playOneMsg(msg_header))
                break;
        }

        closePlay();

        LOG(INFO) << "the message bag play is completed!!!";
    }

    // zmy TODO:play全部或选取部分topic，使用template<typename ...>和std::forward
    void MsgBag::playBack(std::string bag_file, const bool using_terminal, const bool blocking, const bool play_with_time)
    {
        if (playing_)
        {
            LOG(WARNING) << "thers is another play task is runing... !!";
            return;
        }

        if (bag_file.find("/") == std::string::npos)
            bag_file = bag_dir_ + "/" + bag_file;

        playing_ = true;
        play_with_time_ = play_with_time;
        using_terminal_ = using_terminal;
        if (blocking)
            playImp(bag_file);
        else
            play_thread_ = std::async(
                std::launch::async, [bag_file, this]()
                { playImp(bag_file); });
    }

    void MsgBag::closePlay()
    {
        std::lock_guard<std::mutex> lock(play_mutex_);
        if (play_bag_.is_open())
        {
            play_bag_.close();
        }
        playing_ = false;
        speed_ = 1;
        time_changed_ = false;
        pause_ = false;
        play_with_time_ = true;
        switch_time_ = -1;
        is_compress_.clear();
        msg_handles_.clear();
        if (using_terminal_)
            play_operator_.wait();
    }

    void MsgBag::printAllMsgType()
    {
        factory_->printInfo();
    }

    void MsgBag::setEndTime(std::ifstream &bag)
    {
        auto before_pos = bag.tellg();

        std::string header = "";
        std::string msg_str = "";

        bag.seekg(0, std::ios_base::end);
        long long begin = bag.tellg();
        long long end = bag.tellg();

        if (begin < 0)
        {
            LOG(ERROR) << "sbag has error,please check the end time!!!";
        }

        while (factory_->getClassID(msg_str) == -1)
        {
            bag.seekg(begin, std::ios_base::beg);
            bag.seekg(-1, std::ios_base::cur);
            while (bag.peek() != bag.widen('\n'))
            {
                bag.seekg(-1, std::ios_base::cur);
            }
            end = bag.tellg();
            bag.seekg(-1, std::ios_base::cur);
            while (bag.peek() != bag.widen('\n'))
            {
                bag.seekg(-1, std::ios_base::cur);
            }
            bag.seekg(1, bag.cur);
            begin = bag.tellg();

            int size = end - begin;

            header = std::string(size, '\0');
            bag.read(&header[0], size);
            auto vec = splitStr(header, ':');
            if (vec.size() > 1)
                msg_str = vec.at(1);
        }

        end_time_ = static_cast<uint64_t>(std::stoull(splitStr(header, ':').at(0)));

        bag.seekg(before_pos, std::ios_base::beg);
    }

    void MsgBag::setSwitchTime(const uint64_t time)
    {
        std::lock_guard<std::mutex> lock(play_mutex_);
        if (begin_time_ == 0 || end_time_ == 0)
        {
            LOG(WARNING) << "The time of bag has not initialize!!";
            return;
        }
        if (time < begin_time_ || time > end_time_)
        {
            LOG(WARNING) << "The input switch time is out of range !!!";
            return;
        }
        switch_time_ = time;
        time_changed_ = true;
    }

    void MsgBag::setMinDeltaTime(std::ifstream &bag)
    {
        auto before_pos = bag.tellg();
        std::string header = "";
        std::string msg_str = "";
        uint64_t time0 = 0;
        uint64_t time1 = 0;
        while (factory_->getClassID(msg_str) == -1 && !bag.eof())
        {
            getline(bag, header);
            auto vec = splitStr(header, ':');
            if (vec.size() > 1)
                msg_str = splitStr(header, ':').at(1);
        }

        time0 = static_cast<uint64_t>(std::stoull(splitStr(header, ':').at(0)));

        if (bag.eof())
            return;

        while (factory_->getClassID(msg_str) == -1 && !bag.eof())
        {
            getline(bag, header);
            auto vec = splitStr(header, ':');
            if (vec.size() > 1)
                msg_str = splitStr(header, ':').at(1);
        }

        time1 = static_cast<uint64_t>(std::stoull(splitStr(header, ':').at(0)));

        bag.seekg(before_pos, std::ios_base::beg);

        auto delta = time1 - time0;
        min_delta_time_ = delta == 0ul ? min_delta_time_ : delta;
        return;
    }

    void MsgBag::setSpeed(const float speed)
    {
        std::lock_guard<std::mutex> lock(play_mutex_);
        speed_ = speed;
        time_changed_ = true;
    }

    void MsgBag::setPause()
    {
        std::lock_guard<std::mutex> lock(play_mutex_);
        pause_ = !pause_;
        time_changed_ = true;
    }

    bool MsgBag::playOneMsg(std::string &msg_header)
    {
        if (!getline(play_bag_, msg_header))
            return false;
        auto header_vec = splitStr(msg_header, ':');
        if (header_vec.size() < 2)
        {
            LOG(WARNING) << "msg_header is missing !!!";
            return true;
        }
        std::string msg_topic = header_vec.at(1);
        size_t i = factory_->getClassID(msg_topic);
        if (i == -1)
        {
            if (msg_handles_.find(msg_topic) != msg_handles_.end())
                LOG(WARNING) << "msg_header:" << msg_topic << "is missing !!!";
            return true;
        }

        auto msg = Container::createMsg(i);
        auto time = static_cast<uint64_t>(std::stoull(header_vec.at(0)));
        boost::apply_visitor([this, msg_topic, time](auto &&msg)
                             { this->playDetail(std::forward<std::decay_t<decltype(msg)>>(msg), msg_topic, time); },
                             msg);
        return true;
    }

    void MsgBag::getPlayOperator()
    {

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        boost::interprocess::managed_shared_memory managed_shm(boost::interprocess::open_or_create, "bag_play_window_shm", 512);
        boost::interprocess::named_mutex named_mtx(boost::interprocess::open_or_create, "bag_play_window_mutex");
        boost::interprocess::named_condition named_cnd(boost::interprocess::open_or_create, "bag_play_window_cnd");
        boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(named_mtx);
        while (playing_)
        {
            boost::posix_time::ptime time_out =
                boost::posix_time::microsec_clock::universal_time() +
                boost::posix_time::millisec(100);
            if (!named_cnd.timed_wait(lock, time_out))
            {
                if (playing_)
                    break;
                else
                    continue;
            }

            std::pair<int *, std::size_t> p = managed_shm.find<int>("play_operation");
            managed_shm.destroy<int>("play_operation");

            if (p.first)
            {
                switch (*p.first)
                {
                case 120: // x
                    setSpeed(std::max(speed_ - 0.1, 0.2));
                    break;

                case 99: // c
                    setSpeed(speed_ + 0.1);
                    break;

                case 122: // z
                    setSpeed(1);
                    break;

                case 32: // space
                    setPause();
                    break;

                case 115: // s
                    nextStep();
                    break;

                case 113: // q
                    // closePlay();
                    playing_ = false;
                    break;
                }
            }
            if (*p.first == 120 || *p.first == 99 || *p.first == 122)
                float *i = managed_shm.construct<float>("bag_play_speed_feedback")(speed_);
            named_cnd.notify_all();
        }
        int *i = managed_shm.construct<int>("bag_play_close_window")(-100);
        named_cnd.notify_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

} // namespace bag