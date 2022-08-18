/**
 * @file common_type.hpp
 * @brief common structures implement
 *
 * common structures implement.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/8/20
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SRC_COMMON_H
#define SRC_COMMON_H

#ifndef IS_WITHIN_SCOPE
#define IS_WITHIN_SCOPE(n, min, max) (n >= min && n <= max)
#endif

// INCLUDE
#include "point_cloud.hpp"
#include <string>
#include <chrono>
#include <sstream>
#include <fstream>
#include <sys/time.h>
#include <map>
#include <iomanip>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <glog/logging.h>

// CODE
namespace common_func {

    /**
     * @brief get the nanosecond from 00:00, January 1, 1970 to the present
     * @return return int64 microseconds.
     */
    static uint64_t get_time_in_ns() {
        return static_cast<uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    }

    /**
     * @brief get the microseconds from 00:00, January 1, 1970 to the present
     * @return return int64 microseconds.
     */
    static uint64_t get_time_in_us() {
        return get_time_in_ns() / 1000L;
    }

    /**
     * @brief get the milliseconds from 00:00, January 1, 1970 to the present
     * @return return int64 microseconds.
     */
    static uint64_t get_timestamp_in_ms() {
        return static_cast<uint64_t>(get_time_in_ns() / 1000000L);
    }

    /**
     * @brief get the milliseconds from 00:00, January 1, 1970 to the present
     * @return c
     */
    static uint64_t get_time_in_ms() {
        return get_time_in_ns() / 1000000L;
    }

    /**
     * @brief get the seconds from 00:00, January 1, 1970 to the present
     * @return return int64 microseconds.
     */
    static uint64_t get_time_in_s() {
        return get_time_in_ns() / 1000000000L;
    }

    /**
     * \brief get time now to format year-mon-day_hour:min:sec (eg: 2020-08-25_21:26:10)
     * \return date time string
     */
    static std::string getDateTimeStr() {
        struct timeval tv{};
        struct timezone tz{};
        struct tm *tmp;
        gettimeofday(&tv, &tz);
        tmp = localtime(&tv.tv_sec);

        std::ostringstream ss;
//        ss << tmp->tm_year + 1900 << "-" << tmp->tm_mon + 1 << "-" << tmp->tm_mday << "_"
//           << tmp->tm_hour << ":" << tmp->tm_min << ":" << tmp->tm_sec << "." << tv.tv_usec;
        ss << tmp->tm_year + 1900 << std::setfill('0') << std::setw(2) << tmp->tm_mon + 1
           << std::setw(2) <<tmp->tm_mday << "-" <<  std::setw(2) << tmp->tm_hour
           << std::setw(2) << tmp->tm_min <<  std::setw(2) <<  tmp->tm_sec << "." << tv.tv_usec;

        return ss.str();
    }

    /**
     * @brief callback function of std::sort.
     * @param[in] a a float data.
     * @param[in] b a float data.
     * @return if a < b return true, otherwise return false.
     */
    static bool compare_xy(const float &a, const float &b) {
        return a < b;
    }

    /**
     * @brief check if the file exists.
     * @param[in] name file path.
     * @return if the file exists return true, otherwise return false.
     */
    static bool isFileExistent(const std::string& name) {
        return ( access( name.c_str(), F_OK ) != -1 );
    }

    /**
     * @brief get param from param string.
     * @tparam[in] T return param type.
     * @param[in] param_map all of the param.
     * @param[in] param param string name.
     * @param default_value if param not find in param_map. then set param to default value.
     * @return if param not find in param_map. return the default value, otherwise the param value.
     */
    template<typename T>
    inline T GetParamValue(
        const std::map<std::string, std::string> &param_map,
        const std::string& param,
        const T default_value) {

        T ret = default_value;
        auto it = param_map.find(param);

        if (param_map.end() == it){
            LOG(WARNING) << "param " << param << " set default value " << default_value;
            return ret;
        }

        if (typeid(T) == typeid(bool)) {
            std::istringstream(it->second) >> std::boolalpha >> ret;
        } else {
            std::istringstream(it->second) >> ret;
        }

        return ret;
    }

    /**
     * @brief Transform gray image into color image。
     * @param[in] src image source。
     * @param[out] dst image after transform.
     * @param[in] colormap The type of color image you want to convert。
     */
    static void
    GrayMaptoColor(const cv::Mat &src,
                   cv::Mat &dst,
                   const cv::ColormapTypes colormap = cv::COLORMAP_JET) {
        cv::Mat gray = src;

        if (src.empty()) return;

        if (src.type() != CV_8UC1) {
            cv::cvtColor(src, gray, CV_RGB2GRAY);
        }
        // Gray color mapping transformation
        double vmin, vmax, alpha;
        minMaxLoc(gray, &vmin, &vmax);
        alpha = 255.0 / (vmax - vmin);
        gray.convertTo(gray, CV_8U, alpha, -vmin * alpha);
        applyColorMap(gray, dst, colormap);
    }

    /**
     * @brief Transform xyz image to point cloud.
     * @param[in] xyz_img xyz mat. each pixel can be converted into a 3D coordinate point.
     * @param[out] cloud converted point set.
     */
    static void
    xyzImage2PointCloud(const cv::Mat &xyz_img, const PointCloudPtr &cloud) {
        if (xyz_img.type() != CV_16SC3) return;

        int image_height = xyz_img.rows;
        int image_width = xyz_img.cols;

        cloud->resize(image_width * image_height);

        for (int row = 0; row < image_height; row++) {
            for (int col = 0; col < image_width; col++) {
                const int idx = row * image_width + col;
                const auto pixel = xyz_img.at<cv::Vec3s>(row, col);
                cloud->points[idx].x = static_cast<float>(pixel[0]) * 0.001f;
                cloud->points[idx].y = static_cast<float>(pixel[1]) * 0.001f;
                cloud->points[idx].z = static_cast<float>(pixel[2]) * 0.001f;
                cloud->image_indices[idx] = idx;
            }
        }
    }

    static void
    xyzImage2PointCloud(const cv::Mat &xyz_img, const cv::Mat &mask_img, const PointCloudPtr &cloud) {
        if (xyz_img.type() != CV_16SC3 ) {
            return;
        }
        int image_height = xyz_img.rows;
        int image_width = xyz_img.cols;

//        cloud->reserve(image_width * image_height);
        cloud->resize(image_width * image_height);

        for (int row = 0; row < image_height; row++) {
            for (int col = 0; col < image_width; col++) {
                const int idx = row * image_width + col;
                const auto pixel = xyz_img.at<cv::Vec3s>(row, col);
                if(mask_img.at<uint8_t>(row, col) < 100){
                    cloud->points[idx].x = 0;
                    cloud->points[idx].y = 0;
                    cloud->points[idx].z = 0;
                    cloud->image_indices[idx] = idx;
//                    Point3D p(0,0,0);
//                    cloud->push_back(p, idx);
                } else {
                    cloud->points[idx].x = static_cast<float>(pixel[0]) * 0.001f;
                    cloud->points[idx].y = static_cast<float>(pixel[1]) * 0.001f;
                    cloud->points[idx].z = static_cast<float>(pixel[2]) * 0.001f;
                    cloud->image_indices[idx] = idx;
//
//                    Point3D p( static_cast<float>(pixel[0]) * 0.001f,
//                               static_cast<float>(pixel[1]) * 0.001f,
//                               static_cast<float>(pixel[2]) * 0.001f);
//                    cloud->push_back(p, idx);
                }

            }
        }
    }

    template<typename T>
    static std::vector<T>
    splitStringToVector(const std::string &s, const char seperator) {
        std::vector<T> result;
        typedef std::string::size_type string_size;

        string_size i = 0;
        string_size j = 0;
        char c = s[0];
        if (c == '"')  // chip 设置的话一般带“”
            j = 1;
        // LOG(INFO) << "the s is:" << s;
        while (i < s.size()) {
            if (s[i] == seperator || i == s.size() - 1) {
                if (j != i || i == s.size() - 1) {
                    auto len = (s[i] != seperator && i == s.size() - 1) ? (s.size() - j) : i - j;
                    std::string item_s = s.substr(j, len);
                    // LOG(INFO) << "s.substr:" << item_s;
                    if (item_s == "\"") break;
                    try {
                        T ret;
                        if (typeid(T) == typeid(bool)) {
                            std::istringstream(item_s) >> std::boolalpha >> ret;
                        } else {
                            std::istringstream(item_s) >> ret;
                        }
                        result.push_back(ret);
                        // LOG(INFO) << "push back substr " << ret << " size:" <<  result.size(); 
                    } catch (std::exception &e) {
                        LOG(INFO) << "exception:" << e.what() << "," << item_s;
                    }
                }
                j = i + 1;
            }
            i++;
        }
        return result;
    }

static cv::Mat imgRotate(const cv::Mat &matSrc,
                         const float theta,
                         const bool direction) {
    int nRowsSrc = matSrc.rows;
    int nColsSrc = matSrc.cols;

    float rotation_theta = theta;

    if (!direction)
        rotation_theta = static_cast<float>(2.0f * CV_PI - rotation_theta);

    float matRotate[3][3]{
        {std::cos(rotation_theta), -std::sin(rotation_theta), 0},
        {std::sin(rotation_theta), std::cos(rotation_theta),  0},
        {0,                        0,                         1}
    };

    float pt[3][2]{
        {0,                static_cast<float>(nRowsSrc)},
        {static_cast<float>(nColsSrc), static_cast<float>(nRowsSrc)},
        {static_cast<float>(nColsSrc), 0}
    };

    for (auto &i : pt) {
        float x = i[0] * matRotate[0][0] + i[1] * matRotate[1][0];
        float y = i[0] * matRotate[0][1] + i[1] * matRotate[1][1];
        i[0] = x;
        i[1] = y;
    }

    float fMin_x = cv::min(cv::min(cv::min(pt[0][0], pt[1][0]), pt[2][0]), .0f);
    float fMin_y = cv::min(cv::min(cv::min(pt[0][1], pt[1][1]), pt[2][1]), .0f);
    float fMax_x = cv::max(cv::max(cv::max(pt[0][0], pt[1][0]), pt[2][0]), .0f);
    float fMax_y = cv::max(cv::max(cv::max(pt[0][1], pt[1][1]), pt[2][1]), .0f);
    int nRows = cvRound(fMax_y - fMin_y + 0.5) + 1;
    int nCols = cvRound(fMax_x - fMin_x + 0.5) + 1;
    int nMin_x = cvRound(fMin_x + 0.5);
    int nMin_y = cvRound(fMin_y + 0.5);

    cv::Mat matRet(nRows, nCols, matSrc.type(), cv::Scalar(0));
    for (int j = 0; j < nRows; j++) {
        for (int i = 0; i < nCols; i++) {
            int x = (i + nMin_x) * matRotate[0][0] + (j + nMin_y) * matRotate[0][1];
            int y = (i + nMin_x) * matRotate[1][0] + (j + nMin_y) * matRotate[1][1];
            if (x >= 0 && x < nColsSrc && y >= 0 && y < nRowsSrc) {
                matRet.at<uint16_t>(j, i) = matSrc.at<uint16_t>(y, x);
            }
        }
    }
    return matRet;
}

static void transformPoint(const Pose3D &pose3d,
                           const std::vector<Point3D> &input,
                           std::vector<Point3D> &output) {

    double alpha = (pose3d.rotation.x / 180) * M_PI;
    double bate = (pose3d.rotation.y / 180) * M_PI;
    double theta = (pose3d.rotation.z / 180) * M_PI;
    /*
     * |               cos(bate)*cos(theta)                                     cos(bate)*sin(theta)                    -sin(bate)         Cx |
     * | -cos(alpha)*sin(theta)+sin(alpha)*sin(bate)*cos(theta) cos(alpha)*cos(theta)+sin(alpha)*sin(bate)*sin(theta) sin(alpha)*cos(bate) Cy |
     * | sin(alpha)*sin(theta)+cos(alpha)*sin(bate)*cos(theta) -sin(alpha)*cos(theta)+cos(alpha)*sin(bate)*sin(theta) cos(alpha)*cos(bate) Cz |
     * |                       0                                                       0                                      0            1  |
     */

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 0) = cos(bate) * cos(theta);
    transform(0, 1) = cos(bate) * sin(theta);
    transform(0, 2) = -sin(bate);
    transform(0, 3) = pose3d.coordinate.x;
    transform(1, 0) = -cos(alpha) * sin(theta) + sin(alpha) * sin(bate) * cos(theta);
    transform(1, 1) = cos(alpha) * cos(theta) + sin(alpha) * sin(bate) * sin(theta);
    transform(1, 2) = sin(alpha) * cos(bate);
    transform(1, 3) = pose3d.coordinate.y;
    transform(2, 0) = sin(alpha) * sin(theta) + cos(alpha) * sin(bate) * cos(theta);
    transform(2, 1) = -sin(alpha) * cos(theta) + cos(alpha) * sin(bate) * sin(theta);
    transform(2, 2) = cos(alpha) * cos(bate);
    transform(2, 3) = pose3d.coordinate.z;
    transform(3, 0) = 0;
    transform(3, 1) = 0;
    transform(3, 2) = 0;
    transform(3, 3) = 1;

    LOG(INFO) << transform;

    // 记录坐标点到文件
    auto record_to_file = [&](const std::string &path, const std::vector<Point3D> &points) {
        std::ofstream src_stream;
        src_stream.open(path);
        src_stream << points.size();
        for (auto const &p : points)
            src_stream << p.x << " " << p.y << " " << p.z;
        src_stream.close();
    };

    // Record the point before coordinate transformation
    record_to_file("./src_point_cloud.txt", input);

    // 3D coordinate transformation.
    Eigen::Vector4f p1, p2;
    const size_t size = input.size();
    output.resize(size);
    for (size_t i = 0; i < size; ++i) {
        p1[0] = input[i].x;
        p1[1] = input[i].y;
        p1[2] = input[i].z;

        p2 = transform * p1;

        output[i].x = p2[0];
        output[i].y = p2[1];
        output[i].z = p2[2];
    }

    // Record the point after coordinate transformation
    record_to_file("./dst_point_cloud.txt", input);
}


/**
 *
 * @param cloud
 * @param dim
 * @param avg
 * @return
 */
    static
    bool calcPointsXYZAvg(const PointCloudPtr cloud, std::string dim, float& avg){
        const size_t size = cloud->points.size();
        if(0 == size) return false;

        float total = 0;
        if (dim == "x") {
            for (auto point: cloud->points) {
                total += point.x;
            }
        } else if (dim == "y") {
            for (auto point: cloud->points) {
                total += point.y;
            }
        } else if (dim == "z") {
            for (auto point: cloud->points) {
                total += point.z;
            }
        } else {
            return false;
        }
        avg = total/size;
        return true;
    }

}

#endif //SRC_COMMON_H
