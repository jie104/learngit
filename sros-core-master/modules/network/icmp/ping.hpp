//
// ping.cpp
// ~~~~~~~~
//
// Copyright (c) 2003-2018 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <istream>
#include <iostream>
#include <ostream>

#include "icmp_header.hpp"
#include "ipv4_header.hpp"

namespace network {
namespace icmp {

using boost::asio::ip::icmp;
using boost::asio::deadline_timer;
namespace posix_time = boost::posix_time;

class pinger
{
public:
    pinger(boost::asio::io_service& io_service, const char* destination);

    typedef boost::function<void(double value)> PingCallbackFunc_t;
    typedef boost::function<void()> TimeoutCallbackFunc_t;
    
    void setPingCallbackFunc(PingCallbackFunc_t callback);
    void setTimeoutCallbackFunc(TimeoutCallbackFunc_t callback);

private:
    void start_send();
    
    void handle_timeout();
    
    void start_receive();
    
    void handle_receive(std::size_t length);
    
    static unsigned short get_identifier();
    
    icmp::resolver resolver_;
    icmp::endpoint destination_;
    icmp::socket socket_;
    deadline_timer timer_;
    unsigned short sequence_number_;
    posix_time::ptime time_sent_;
    boost::asio::streambuf reply_buffer_;
    std::size_t num_replies_;
    
    PingCallbackFunc_t ping_callback_f_;
    TimeoutCallbackFunc_t timeout_callback_f_;
};

}
}

