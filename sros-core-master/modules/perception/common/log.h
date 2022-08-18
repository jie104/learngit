/**
 * @file log.h
 * @brief log class
 *
 * The log class encapsulates the interface functions required by the glog system,
 * if you want to use，If you want to use the log system, you first need to refer
 * to the log.h header file and then call the init function.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/8/25
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SRC_UTIL_H
#define SRC_UTIL_H

// INCLUDE
#include <mutex>

//CODE
/**
 * \brief * Wrapper to ensure google logging only initializes once. Secondarily, this
 * also acts as a very thin abstraction layer over the logging subsystem.
 *
 * \author zhangxu@standard-robots.com
 * \date 2020-08-25 21:32:08
 */
class Logging {

public:

    /**
     * \brief Function used to initialize the library's logging subsystem. This function
     *        is thread safe _and_ idempotent. However, it must be called _at least
     *        once_ prior to writing log messages.
     * \param[in] log_path log file generate directory.
     * \param[in] file_name log file name.
     * \Author: zhangxu@standard-robots.com
     * \Date: 2020-08-25 21:31:11
     */
    static void init(const std::string &log_path,
              const std::string &file_name);

    /**
     * \brief Function used to close the logging.
     */
    static void shutdownLog();

private:
    /**
      * \brief Actually runs the one-time initialization code.
      * \Author: zhangxu@standard-robots.com
      * \Date: 2020-08-25 21:29:46
      */
    static void _init();

     /**
      * \brief Flag indicating the initialization state of the logging subsystem.
      * \Author: zhangxu@standard-robots.com
      * \Date: 2020-08-25 21:30:15
      */
    static std::once_flag init_;
    static std::string log_path_;
    static std::string file_name_;

}; // end: class Logging



#endif //SRC_UTIL_H
