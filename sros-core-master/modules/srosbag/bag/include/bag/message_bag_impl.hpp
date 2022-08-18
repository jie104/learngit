/**
 * @file message_bag_impl.hpp
 * @author zmy (626670628@qq.com)
 * @brief msgbag模板函数实现
 * @version 0.1
 * @date 2022-04-14
 * 
 * 
 */

#ifndef MESSAGE_BAG_IMPL_HPP
#define MESSAGE_BAG_IMPL_HPP

namespace bag
{
    template <typename Msg>
    auto MsgBag::dumpMsg(const Msg &msg, const std::string &topic)
    {
        std::string msg_topic = topic;
        if (use_msg_[Container::typeToPos<Msg>()] == true && std::find(record_topics_.begin(), record_topics_.end(), msg_topic) != record_topics_.end())
        {
            auto time = sros::core::util::get_timestamp_in_us();
            if (!record_bag_.is_open())
            {
                std::lock_guard<std::mutex> lock(record_mutex_);
                if (!record_bag_.is_open())
                    createBag(time);
            }
            thread_pool_->submit([this, msg = std::move(msg), topic = std::move(msg_topic), time]()
                                 { save<Msg>(msg, topic, time); });
        }
    }

    template <typename Msg, std::enable_if_t<!std::is_base_of<::google::protobuf::Message, Msg>::value> *>
    auto MsgBag::save(const Msg &msg, const std::string &topic, const uint64_t time)
    {
        record_mutex_.lock();
        write_cache_.push(time);
        record_mutex_.unlock();

        std::ostringstream oss;
        if (factory_->isCompress(topic))
        {
            {
                boost::iostreams::filtering_stream<boost::iostreams::output> out;
                out.push(boost::iostreams::zlib_compressor(boost::iostreams::zlib::best_speed));
                out.push(oss);
#ifdef USE_EOS
                eos::portable_oarchive oa(out);
#else
                boost::archive::text_oarchive oa(out);
#endif
                oa << msg;
            }
        }
        else
        {

#ifdef USE_EOS
            eos::portable_oarchive oa(oss);
#else
            boost::archive::text_oarchive oa(oss);
#endif
            oa << msg;
        }
        {
            std::unique_lock<std::mutex> lk(record_mutex_);
            if (write_cache_.size() > 1)
            {
                write_cv_.wait(lk, [this, time]()
                               { return write_cache_.top() >= time; });
            }

            record_bag_ << time << ":" << topic << std::endl;
            record_bag_ << oss.str() << end_mark_ << std::endl;
            write_cache_.pop();
            lk.unlock();
            write_cv_.notify_all();
        }
        return;
    }

    template <typename Msg, std::enable_if_t<std::is_base_of<::google::protobuf::Message, Msg>::value ||
                                             std::is_base_of<::google::protobuf::MessageLite, Msg>::value> *>
    auto MsgBag::save(const Msg &msg, const std::string &topic, const uint64_t time)
    {
        // zmy XXX:未测试

        record_mutex_.lock();
        write_cache_.push(time);
        record_mutex_.unlock();

        if (factory_->isCompress(topic))
        {
            std::string data;
            msg.SerializeToString(&data);

            std::stringstream compressed;
            std::stringstream origin(data);

            boost::iostreams::filtering_streambuf<boost::iostreams::input> out;
            out.push(boost::iostreams::zlib_compressor(boost::iostreams::zlib::best_speed));
            out.push(origin);
            boost::iostreams::copy(out, compressed);

            {
                std::unique_lock<std::mutex> lk(record_mutex_);
                if (write_cache_.size() > 1)
                {
                    write_cv_.wait(lk, [this, time]()
                                   { return write_cache_.top() >= time; });
                }
                record_bag_ << time << ":" << topic << std::endl;
                record_bag_ << compressed.str() << end_mark_ << std::endl;
                write_cache_.pop();
                lk.unlock();
                write_cv_.notify_all();
            }
        }
        else
        {
            std::string data;
            msg.SerializeToString(&data);

            {

                std::unique_lock<std::mutex> lk(record_mutex_);
                write_cv_.wait(lk, [this, time]()
                               { return write_cache_.top() >= time; });
                record_bag_ << time << ":" << topic << std::endl;
                record_bag_ << data << end_mark_ << std::endl;
                write_cache_.pop();
                lk.unlock();
                write_cv_.notify_all();
            }
        }
    }

    template <typename Msg>
    auto MsgBag::setMsgHandle(const std::function<void(const Msg &)> &handle, const std::string &topic)
    {
        if (topic == "")
            return;
        if (msg_handles_.find(topic) == msg_handles_.end())
        {
            msg_handles_[topic] = handle;
        }
        else
        {
            msg_handles_[topic] = handle;
            LOG(WARNING) << "Pay attention to The handle of " << topic << " has be replaced!!!";
        }
        return;
    }

    template <typename Msg>
    auto MsgBag::handleMsg(Msg &msg, const std::string &msg_header)
    {
        if (msg_handles_.find(msg_header) != msg_handles_.end())
        {
            auto &any_handle = msg_handles_[msg_header];
            if (!any_handle.empty())
            {
                auto handle_msg = boost::any_cast<std::function<void(const Msg &)>>(any_handle);
                handle_msg(msg);
            }
            else
            {
                LOG(WARNING) << "not set handle message function for " << msg_header << "!!!";
            }
        }
    }

    template <typename Msg, std::enable_if_t<!std::is_base_of<::google::protobuf::Message, Msg>::value> *>
    auto MsgBag::loadMsg(Msg &msg, const std::string &topic)
    {
        if (is_compress_.at(topic))
        {
            std::string msg_data;
            getline(play_bag_, msg_data);
            auto pos = msg_data.rfind(end_mark_);
            while (pos == std::string::npos || (msg_data.size() - pos != end_mark_.size()))
            {
                std::string app;
                getline(play_bag_, app);
                msg_data = msg_data + '\n' + app;
                pos = msg_data.rfind(end_mark_);
            }
            msg_data.erase(pos);

            std::istringstream iss(msg_data);
            boost::iostreams::filtering_stream<boost::iostreams::input> in;
            in.push(boost::iostreams::zlib_decompressor());
            in.push(iss);
#ifdef USE_EOS
            eos::portable_iarchive ia(in);
#else
            boost::archive::text_iarchive ia(in);
#endif
            ia >> msg;
        }
        else
        {
#ifdef USE_EOS
            eos::portable_iarchive ia(play_bag_);
#else
            boost::archive::text_iarchive ia(play_bag_);
#endif
            ia >> msg;
            //移除流中的msgType
            std::string app;
            getline(play_bag_, app);
            if (app != end_mark_)
            {
                LOG(WARNING) << "The postfix of " << topic << " is not match!!!";
            }
        }

        return;
    }

    template <typename Msg, std::enable_if_t<std::is_base_of<::google::protobuf::Message, Msg>::value ||
                                             std::is_base_of<::google::protobuf::MessageLite, Msg>::value> *>
    auto MsgBag::loadMsg(Msg &msg, const std::string &topic)
    {
        // zmy XXX:未测试
        std::string msg_data;
        getline(play_bag_, msg_data);
        auto pos = msg_data.rfind(end_mark_);
        while (pos == std::string::npos || (msg_data.size() - pos != end_mark_.size()))
        {
            std::string app;
            getline(play_bag_, app);
            msg_data = msg_data + '\n' + app;
            pos = msg_data.rfind(end_mark_);
        }
        msg_data.erase(pos);

        if (is_compress_.at(topic))
        {
            std::stringstream compressed(msg_data);
            std::stringstream decompressed;

            boost::iostreams::filtering_streambuf<boost::iostreams::input> out;
            out.push(boost::iostreams::zlib_decompressor());
            out.push(compressed);
            boost::iostreams::copy(out, decompressed);

            msg.ParseFromString(decompressed.str());
        }
        else
        {
            msg.ParseFromString(msg_data);
        }

        // zmy TODO: return time_stamp
        //  return time_stamp
        return;
    }

    template <typename Msg>
    auto MsgBag::playDetail(Msg &&msg, const std::string &msg_header, const uint64_t stamp)
    {
        if (stamp < begin_time_ || stamp > end_time_)
            return;
        static bool jump_forward = false;
        static bool jump_backward = false;
        if (switch_time_ == -1)
        {
            loadMsg<Msg>(msg, msg_header);

            // zmy TODO: 时间有效性判断
            if (time_changed_)
            {
                time_marker_ = sros::core::util::get_timestamp_in_us() - static_cast<double>((stamp - begin_time_) / speed_);
                time_changed_ = false;
            }

            if (play_with_time_)
            {
                std::this_thread::sleep_until(std::chrono::time_point<
                                              std::chrono::high_resolution_clock, std::chrono::duration<double, std::micro>>(
                    std::chrono::duration<double, std::micro>(static_cast<double>((stamp - begin_time_) / speed_ + time_marker_))));
            }

            handleMsg<Msg>(msg, msg_header);

            if (end_time_ != 0)
                LOG(INFO) << "Bag Time:" << stamp
                          << "  Duration: " << (stamp - begin_time_) / 1e6 << " / "
                          << (end_time_ - begin_time_) / 1e6;
        }
        else if (!jump_forward && static_cast<int64_t>(stamp) - switch_time_ > 1.5 * min_delta_time_)
        {
            int pos = (static_cast<double>(switch_time_ - begin_time_) * 0.8) / (end_time_ - begin_time_) * (end_cur_ - begin_cur_);
            play_bag_.seekg(pos, std::ios_base::beg);
            time_changed_ = true;
            jump_backward = true;
            return;
        }
        else if (!jump_backward && switch_time_ - static_cast<int64_t>(stamp) > 1.5 * min_delta_time_)
        {
            time_changed_ = true;
            jump_forward = true;
            return;
        }
        else
        {
            switch_time_ = -1;
            time_changed_ = true;
            jump_forward = false;
            jump_backward = false;
            return;
        }
    }

    template <typename Msg>
    auto MsgBag::writeMsgHeader(const std::string &topic)
    {
        LOG(INFO) << topic << "\n";
        // std::lock_guard<std::mutex> lock(record_mutex_);
        record_bag_ << topic << "," << factory_->isCompress(topic) << ';';
    }

    template <typename Msg>
    auto MsgBag::setTime(Msg &&msg, const std::string &header, uint64_t &time)
    {
        time = loadMsg<Msg>(msg, header);
    }
}
#endif