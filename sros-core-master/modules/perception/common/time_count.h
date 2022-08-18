#ifndef TIME_COUNT_HH
#define TIME_COUNT_HH

#include <glog/logging.h>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <algorithm>


class TimeCount {
private:
    struct TimeDetail {
        unsigned int num;
        double min;
        double max;
        double mean;
        std::vector<double> values;
    };

    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;
    using TimeDuration = std::chrono::nanoseconds;

    static std::unordered_map<std::string, TimePoint> g_tick_time_points;
    static std::unordered_map<std::string, TimeDetail> g_details;

    static inline std::string toString(const double count) {
        std::ostringstream oss;
        oss << std::setprecision(3) << count;
        return oss.str();
    }

    static inline std::tuple<double, double> getMedianAndSigma(std::vector<double> &values) {
        std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
        const double median = values[values.size() / 2];
        std::vector<double> bias(values.size());
        for (int i = 0; i < values.size(); i++) {
            bias[i] = std::abs(values[i] - median);
        }
        std::nth_element(bias.begin(), bias.begin() + values.size() / 2, bias.end());
        return std::make_tuple(median, 1.4826 * bias[values.size() / 2]);
    }

    static inline std::string toStringDuration(double count) {
//        static const std::string duration_name[7] = {"纳秒", "微秒", "毫秒", "秒", "分", "时", "天"};
        static const std::string duration_name[7] = {"ns", "us", "ms", "s", "m", "h", "t"};
        if (count < 1000.0) return toString(count) + duration_name[0];
        count /= 1000.0;
        if (count < 1000.0) return toString(count) + duration_name[1];
        count /= 1000.0;
        if (count < 1000.0) return toString(count) + duration_name[2];
        count /= 1000.0;
        if (count < 60.0) return toString(count) + duration_name[3];
        long int long_count = static_cast<long int>(count);
        std::string fixed = std::to_string(long_count % 60l) + duration_name[3];
        long_count /= 60l;
        if (long_count < 60l) return std::to_string(long_count) + duration_name[4] + fixed;
        fixed = std::to_string(long_count % 60l) + duration_name[4] + fixed;
        long_count /= 60l;
        if (long_count < 24l) return std::to_string(long_count) + duration_name[5] + fixed;
        fixed = std::to_string(long_count % 24l) + duration_name[5] + fixed;
        long_count /= 24l;
        return std::to_string(long_count) + duration_name[6] + fixed;
    }

    static inline std::string toStringDuration(const TimeDuration &duration) {
        return toStringDuration(static_cast<double>(duration.count()));
    }

public:
    //开始
    static inline void tick(const std::string &title = std::string()) {
        g_tick_time_points[title] = Clock::now();
    }

    //停止
    static inline void tock(const std::string &title = std::string()) {
        auto iter = g_tick_time_points.find(title);
        if (iter == g_tick_time_points.end()) {
            LOG(ERROR) << title << "未设置起始时间点!!!";
        }
        TimeDuration duration = std::chrono::duration_cast<TimeDuration>(Clock::now() - iter->second);
        LOG(INFO) << title << "耗时为: " << toStringDuration(duration);
        g_tick_time_points.erase(iter);
    }

    //停止并记录
    static inline void tockAndRecord(const std::string &title) {
        auto iter = g_tick_time_points.find(title);
        if (iter == g_tick_time_points.end()) {
            LOG(ERROR) << title << "未设置起始时间点!!!";
        }
        TimeDuration duration = std::chrono::duration_cast<TimeDuration>(Clock::now() - iter->second);
        const double duration_count = static_cast<double>(duration.count());
        g_tick_time_points.erase(iter);
        auto detail_iter = g_details.find(title);
        if (detail_iter != g_details.end()) {
            detail_iter->second.num++;
            const double tmp =
                (duration_count - detail_iter->second.mean) / static_cast<double>(detail_iter->second.num);
            detail_iter->second.mean += tmp;
            detail_iter->second.min =
                duration_count < detail_iter->second.min ? duration_count : detail_iter->second.min;
            detail_iter->second.max =
                duration_count > detail_iter->second.max ? duration_count : detail_iter->second.max;
            detail_iter->second.values.push_back(duration_count);
        } else {
            g_details[title] = {1, duration_count, duration_count, duration_count, {duration_count}};
        }
    }

    //停止并记录并输出
    static inline void tockAndRecordAndOutput(const std::string &title) {
        auto iter = g_tick_time_points.find(title);
        if (iter == g_tick_time_points.end()) {
            LOG(ERROR) << title << "未设置起始时间点!!!";
        }
        TimeDuration duration = std::chrono::duration_cast<TimeDuration>(Clock::now() - iter->second);
        const double duration_count = static_cast<double>(duration.count());
        LOG(INFO) << title << "耗时为: " << toStringDuration(duration);
        g_tick_time_points.erase(iter);
        auto detail_iter = g_details.find(title);
        if (detail_iter != g_details.end()) {
            detail_iter->second.num++;
            const double tmp =
                (duration_count - detail_iter->second.mean) / static_cast<double>(detail_iter->second.num);
            detail_iter->second.mean += tmp;
            detail_iter->second.min =
                duration_count < detail_iter->second.min ? duration_count : detail_iter->second.min;
            detail_iter->second.max =
                duration_count > detail_iter->second.max ? duration_count : detail_iter->second.max;
            detail_iter->second.values.push_back(duration_count);
        } else {
            g_details[title] = {1, duration_count, duration_count, duration_count, {duration_count}};
        }
        LOG(INFO) << title << "平均耗时为: " << toStringDuration(g_details[title].mean);
    }

    //输出平均时长
    static inline void output(const std::string &title) {
        auto detail_iter = g_details.find(title);
        if (detail_iter != g_details.end()) {
            double median, sigma;
            std::tie(median, sigma) = getMedianAndSigma(detail_iter->second.values);
            std::array<unsigned int, 8> nums;
            nums.fill(0);
            for (const auto &value : detail_iter->second.values) {
                if (value < median - 3. * sigma) {
                    ++nums[0];
                } else if (value < median - 2. * sigma) {
                    ++nums[1];
                } else if (value < median - sigma) {
                    ++nums[2];
                } else if (value < median) {
                    ++nums[3];
                } else if (value < median + sigma) {
                    ++nums[4];
                } else if (value < median + 2. * sigma) {
                    ++nums[5];
                } else if (value < median + 3. * sigma) {
                    ++nums[6];
                } else {
                    ++nums[7];
                }
            }
            int start_index = 0, end_index = 7;
            for (int i = 0; i < 8; i++) {
                if (detail_iter->second.min < median + sigma * (static_cast<double>(i) - 3.)) {
                    start_index = i;
                    break;
                }
            }
            for (int i = 7; i >= 0; i--) {
                if (detail_iter->second.max > median + sigma * (static_cast<double>(i) - 4.)) {
                    end_index = i;
                    break;
                }
            }
            std::string histogram;
            const unsigned int max_bar = 40;
            for (int i = start_index; i <= end_index; i++) {
                const auto bar = (nums[i] * max_bar + detail_iter->second.num / 2) / detail_iter->second.num;
                for (int j = 0; j != max_bar; j++) {
                    histogram += (j < (max_bar - bar)) ? " " : "#";
                }
                if (i == start_index) {
                    histogram += " [" + toStringDuration(detail_iter->second.min);
                } else {
                    histogram += " [" + toStringDuration(median + sigma * (static_cast<double>(i) - 4.));
                }
                if (i == end_index) {
                    histogram += ", " + toStringDuration(detail_iter->second.max) + "]: ";
                } else {
                    histogram += ", " + toStringDuration(median + sigma * (static_cast<double>(i) - 3.)) + "): ";
                }
                const double percentage =
                    static_cast<double>(nums[i]) / static_cast<double>(detail_iter->second.num) * 100.;
                histogram += std::to_string(nums[i]) + "/" + std::to_string(detail_iter->second.num) + "(" +
                             toString(percentage) + "%) \n";
            }
            LOG(INFO) << title << "平均耗时为: " << toStringDuration(detail_iter->second.mean)
                      << ", 最小耗时为: " << toStringDuration(detail_iter->second.min)
                      << ", 最大耗时为: " << toStringDuration(detail_iter->second.max)
                << ", 总耗时为: " << toStringDuration(detail_iter->second.mean * detail_iter->second.num) << std::endl
                      << histogram;
        } else {
            LOG(INFO) << title << "未记录过!!!";
        }
    }

    //清除之前的统计
    static inline void clear(const std::string &title) {
        auto detail_iter = g_details.find(title);
        if (detail_iter != g_details.end()) {
            g_details.erase(detail_iter);
        } else {
            LOG(INFO) << title << "没有统计过!!!";
        }
    }

    //清除之前所有的统计
    static void clearAll() { g_details.clear(); }
};

#endif //TIME_COUNT_HH