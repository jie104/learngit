/**
 * @file run_logger.h
 *
 * @author lhx
 * @date 2018年4月25日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef CORE_LOG_RUN_LOGGER_H_
#define CORE_LOG_RUN_LOGGER_H_

#include "../db/db.h"

#include "../util/time.h"

namespace sros {
namespace core {

typedef std::map<std::string, int> LoggerCache;

class RunLogger {
 public:
    static RunLogger &getInstance();

    int newRun(const std::string &sros_version_str, const std::string src_version_str);

    uint32_t keepAlive(uint32_t mileage);

    int getTotalRunTimeInSeconds();

    int getTotalMileage();

    int getTotalBootTimes();

    // 当开机后读取到的里程不为0时，以该值作为零点
    void setMileageOffset(uint32_t zero_offset);

 private:
    RunLogger();
    RunLogger(RunLogger const &);
    RunLogger &operator=(RunLogger const &);

    ~RunLogger();

    int id_;

    SQLite::Database *db_;

    LoggerCache cache_;

    uint32_t mileage_zero_offset_ = 0;  // 里程零点偏移
};

}  // namespace core
}  // namespace sros

#endif  // CORE_LOG_RUN_LOGGER_H_
