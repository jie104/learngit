/*
 * File:   AsyncSerial.cpp
 * Author: Terraneo Federico
 * Distributed under the Boost Software License, Version 1.0.
 * Created on September 7, 2009, 10:46 AM
 *
 * v1.02: Fixed a bug in BufferedAsyncSerial: Using the default constructor
 * the callback was not set up and reading didn't work.
 *
 * v1.01: Fixed a bug that did not allow to reopen a closed serial port.
 *
 * v1.00: First release.
 *
 * IMPORTANT:
 * On Mac OS X boost asio's serial ports have bugs, and the usual implementation
 * of this class does not work. So a workaround class was written temporarily,
 * until asio (hopefully) will fix Mac compatibility for serial ports.
 *
 * Please note that unlike said in the documentation on OS X until asio will
 * be fixed serial port *writes* are *not* asynchronous, but at least
 * asynchronous *read* works.
 * In addition the serial port open ignores the following options: parity,
 * character size, flow, stop bits, and defaults to 8N1 format.
 * I know it is bad but at least it's better than nothing.
 *
 */

#include "async_serial.h"

#include <glog/logging.h>
#include <algorithm>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <string>
#include "core/util/utils.h"

#include "rs485_gpio.h"

using namespace std;
using namespace boost;

//
// Class AsyncSerial
//

class AsyncSerialImpl : private boost::noncopyable {
 public:
    AsyncSerialImpl() : io(), port(io), backgroundThread(), open(false), error(false) {}

    boost::asio::io_service io;       ///< Io service object
    boost::asio::serial_port port;    ///< Serial port object
    boost::thread backgroundThread;   ///< Thread that runs read/write operations
    bool open;                        ///< True if port open
    bool error;                       ///< Error flag
    mutable boost::mutex errorMutex;  ///< Mutex for access to error

    /// Data are queued here before they go in writeBuffer
    std::vector<char> writeQueue;
    std::vector<char> writeBuffer;                 ///< Data being written
    size_t writeBufferSize = 0;                        ///< Size of writeBuffer
    boost::mutex writeQueueMutex;                  ///< Mutex for access to writeQueue
    char readBuffer[AsyncSerial::readBufferSize];  ///< data being read

    /// Read complete callback
    boost::function<void(const char *, size_t)> callback;
};

AsyncSerial::AsyncSerial() : pimpl(new AsyncSerialImpl) {}

AsyncSerial::AsyncSerial(const std::string &devname, unsigned int baud_rate, bool enable_rs485_mode,
                         asio::serial_port_base::parity opt_parity, asio::serial_port_base::character_size opt_csize,
                         asio::serial_port_base::flow_control opt_flow, asio::serial_port_base::stop_bits opt_stop)
    : pimpl(new AsyncSerialImpl) {
    open(devname, baud_rate, enable_rs485_mode, opt_parity, opt_csize, opt_flow, opt_stop);
}

void AsyncSerial::open(const std::string &devname, unsigned int baud_rate, bool enable_rs485_mode,
                       asio::serial_port_base::parity opt_parity, asio::serial_port_base::character_size opt_csize,
                       asio::serial_port_base::flow_control opt_flow, asio::serial_port_base::stop_bits opt_stop) {
    if (isOpen()) close();

    enable_rs485_mode_ = enable_rs485_mode;
    strDevName = devname;

    if (enable_rs485_mode_) {
        init_rs485_gpio();
    }

    setErrorStatus(true);  // If an exception is thrown, error_ remains true
    try {
        pimpl->port.open(devname);
    } catch (...) {
        pimpl->open = false;
        return;
    }
    pimpl->port.set_option(asio::serial_port_base::baud_rate(baud_rate));
    pimpl->port.set_option(opt_parity);
    pimpl->port.set_option(opt_csize);
    pimpl->port.set_option(opt_flow);
    pimpl->port.set_option(opt_stop);

    LOG(INFO) << "[SERIAL] " << devname << " opened, " << baud_rate;

    // This gives some work to the io_service before it is started
    pimpl->io.post(boost::bind(&AsyncSerial::doRead, this));

    thread t(boost::bind(&asio::io_service::run, &pimpl->io));
    pimpl->backgroundThread.swap(t);
    setErrorStatus(false);  // If we get here, no error
    pimpl->open = true;     // Port is now open
}

bool AsyncSerial::isOpen() const { return pimpl->open; }

bool AsyncSerial::errorStatus() const {
    lock_guard<mutex> l(pimpl->errorMutex);
    return pimpl->error;
}

void AsyncSerial::close() {
    if (enable_rs485_mode_) {
        release_rs485_gpio();
    }

    if (!isOpen()) return;

    pimpl->open = false;
    pimpl->io.post(boost::bind(&AsyncSerial::doClose, this));
    pimpl->backgroundThread.join();
    pimpl->io.reset();
    if (errorStatus()) {
        throw(boost::system::system_error(boost::system::error_code(), "Error while closing the device"));
    }
}

void AsyncSerial::write(const char *data, size_t size) {
    {
        lock_guard<mutex> l(pimpl->writeQueueMutex);
        pimpl->writeQueue.insert(pimpl->writeQueue.end(), data, data + size);
    }
    pimpl->io.post(boost::bind(&AsyncSerial::doWrite, this));
}

void AsyncSerial::write(const std::vector<char> &data) {
    {
        lock_guard<mutex> l(pimpl->writeQueueMutex);
        pimpl->writeQueue.insert(pimpl->writeQueue.end(), data.begin(), data.end());
    }
    pimpl->io.post(boost::bind(&AsyncSerial::doWrite, this));
}

void AsyncSerial::writeString(const std::string &s) {
    {
        lock_guard<mutex> l(pimpl->writeQueueMutex);
        pimpl->writeQueue.insert(pimpl->writeQueue.end(), s.begin(), s.end());
    }
    pimpl->io.post(boost::bind(&AsyncSerial::doWrite, this));
}

AsyncSerial::~AsyncSerial() {
    if (isOpen()) {
        try {
            close();
        } catch (...) {
            // Don't throw from a destructor
        }
    }
}

void AsyncSerial::doRead() {
    pimpl->port.async_read_some(
        asio::buffer(pimpl->readBuffer, readBufferSize),
        boost::bind(&AsyncSerial::readEnd, this, asio::placeholders::error, asio::placeholders::bytes_transferred));
}

void AsyncSerial::readEnd(const boost::system::error_code &error, size_t bytes_transferred) {
    if (error) {
        LOG(ERROR) << "read error " << error;
#ifdef __APPLE__
        if (error.value() == 45) {
            // Bug on OS X, it might be necessary to repeat the setup
            // http://osdir.com/ml/lib.boost.asio.user/2008-08/msg00004.html
            doRead();
            return;
        }
#endif  //__APPLE__
        // error can be true even because the serial port was closed.
        // In this case it is not a real error, so ignore
        if (isOpen()) {
            doClose();
            setErrorStatus(true);
        }
    } else {
        if (pimpl->callback) pimpl->callback(pimpl->readBuffer, bytes_transferred);
        doRead();
    }
}

void AsyncSerial::doWrite() {
    // If a write operation is already in progress, do nothing
    if (pimpl->writeBufferSize == 0) {
        lock_guard<mutex> l(pimpl->writeQueueMutex);
        if (pimpl->writeQueue.empty()) {
            return;
        }
        pimpl->writeBufferSize = pimpl->writeQueue.size();
        pimpl->writeBuffer.resize(pimpl->writeQueue.size());
        std::copy(pimpl->writeQueue.begin(), pimpl->writeQueue.end(), pimpl->writeBuffer.begin());
        pimpl->writeQueue.clear();

        if (enable_rs485_mode_) {
            // 开始发送，切换到发送模式
            toggle_rs485_mode(RS485_TX);
        }

        async_write(pimpl->port, asio::buffer(pimpl->writeBuffer.data(), pimpl->writeBufferSize),
                    boost::bind(&AsyncSerial::writeEnd, this, asio::placeholders::error));
    }
}

void AsyncSerial::writeEnd(const boost::system::error_code &error) {
    if (!error) {
        lock_guard<mutex> l(pimpl->writeQueueMutex);
        if (pimpl->writeQueue.empty()) {
            pimpl->writeBufferSize = 0;

            if (enable_rs485_mode_) {
                // 保证数据已经通过串口芯片发送出去再切换状态，否则会导致数据错误
                // http://stackoverflow.com/questions/30078415/what-means-blocking-for-boostasiowrite
                // https://github.com/stephane/libmodbus/wiki/Beaglebone-RS485-cape
                tcdrain(pimpl->port.native_handle());
                // 发送完成，切换到接收模式
                toggle_rs485_mode(RS485_RX);
            }

            return;
        }
        pimpl->writeBufferSize = pimpl->writeQueue.size();
        pimpl->writeBuffer.resize(pimpl->writeQueue.size());
        copy(pimpl->writeQueue.begin(), pimpl->writeQueue.end(), pimpl->writeBuffer.begin());
        pimpl->writeQueue.clear();
        async_write(pimpl->port, asio::buffer(pimpl->writeBuffer.data(), pimpl->writeBufferSize),
                    boost::bind(&AsyncSerial::writeEnd, this, asio::placeholders::error));
    } else {
        LOG(ERROR) << "write error " << error;
        setErrorStatus(true);
        doClose();
    }
}

void AsyncSerial::doClose() {
    LOG(ERROR) << "doClose!";
    boost::system::error_code ec;
    pimpl->port.cancel(ec);
    if (ec) setErrorStatus(true);
    pimpl->port.close(ec);
    if (ec) setErrorStatus(true);
}

void AsyncSerial::setErrorStatus(bool e) {
    lock_guard<mutex> l(pimpl->errorMutex);
    pimpl->error = e;
}

void AsyncSerial::setReadCallback(const boost::function<void(const char *, size_t)> &callback) {
    pimpl->callback = callback;
}

void AsyncSerial::clearReadCallback() { pimpl->callback.clear(); }

//
// Class CallbackAsyncSerial
//

CallbackAsyncSerial::CallbackAsyncSerial() : AsyncSerial() {}

CallbackAsyncSerial::CallbackAsyncSerial(const std::string &devname, unsigned int baud_rate,
                                         asio::serial_port_base::parity opt_parity,
                                         asio::serial_port_base::character_size opt_csize,
                                         asio::serial_port_base::flow_control opt_flow,
                                         asio::serial_port_base::stop_bits opt_stop)
    : AsyncSerial(devname, baud_rate, false, opt_parity, opt_csize, opt_flow, opt_stop) {}

void CallbackAsyncSerial::setCallback(const boost::function<void(const char *, size_t)> &callback) {
    setReadCallback(callback);
}

void CallbackAsyncSerial::clearCallback() { clearReadCallback(); }

CallbackAsyncSerial::~CallbackAsyncSerial() { clearReadCallback(); }
