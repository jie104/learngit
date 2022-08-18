//
// Created by lhx on 16-4-24.
//

#include "pose_saver.h"

#include <fstream>
#include <thread>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include "core/log_helper.hpp"

const std::string OLD_SAVER_FILE_NAME = "/sros/cfg/pose_saver.dat";

const std::string SAVER_FILE_NAME = "/sros/cfg/pose_record.dat";
const std::string SAVER_FILE_NAME_SHADOW = "/sros/cfg/pose_record_shadow.dat";

PoseSaver::PoseSaver() {

}

PoseSaver::~PoseSaver() {

}


void PoseSaver::save(const sros::core::Pose &pose, const std::string &map_name, uint16_t station_no) {
    baseSave(SAVER_FILE_NAME, pose, map_name, station_no);

    // 调用sync保证文件写入磁盘
    std::thread t([=] {
        //system("sync");
        baseSave(SAVER_FILE_NAME_SHADOW, pose, map_name, station_no); // 备份
        //system("sync");
    });
    t.detach(); // 脱离当前线程，执行完成后自动退出
}

bool PoseSaver::load(sros::core::Pose &pose, std::string &map_name, uint16_t &station_no) {
    namespace fs = boost::filesystem;

    if (fs::exists(fs::path(SAVER_FILE_NAME))) {
        if (baseLoad(SAVER_FILE_NAME, pose, map_name, station_no)) {
            return true;
        }

        // 正常的加载失败了就加载备份的
        return baseLoad(SAVER_FILE_NAME_SHADOW, pose, map_name, station_no);
    }

    return oldLoad(pose, map_name, station_no);
}

void
PoseSaver::baseSave(const std::string &file_name, const sros::core::Pose &pose, const std::string &map_name,
                    uint16_t station_no) {
    std::ofstream out(file_name, std::ifstream::out | std::ifstream::binary);

    if (out.fail()) {
        LOG(ERROR) << "failed to open PoseSaver file : " << file_name;
        return;
    }

    uint16_t map_name_len = map_name.length();

    out.write(reinterpret_cast<const char *>(&pose.x()), sizeof(pose.x()));
    out.write(reinterpret_cast<const char *>(&pose.y()), sizeof(pose.y()));
    out.write(reinterpret_cast<const char *>(&pose.yaw()), sizeof(pose.yaw()));
    out.write(reinterpret_cast<const char *>(&station_no), sizeof(station_no));
    out.write(reinterpret_cast<const char *>(&map_name_len), sizeof(map_name_len));
    out.write(map_name.c_str(), map_name_len);

    out.close();
}

bool
PoseSaver::baseLoad(const std::string &file_name, sros::core::Pose &pose, std::string &map_name, uint16_t &station_no) {
    std::ifstream in(file_name, std::ifstream::in | std::ifstream::binary);

    if (in.fail()) {
        LOG(ERROR) << "failed to open PoseSaver file : " << file_name;
        return false;
    }

    // 判断文件大小是否合法
    in.seekg(0, std::ios_base::end);
    long size = in.tellg();

    if (size == 0) {
        LOG(ERROR) << "PoseSaver file is empty : " << file_name;
        return false;
    }
    in.seekg(std::ios_base::beg); // 恢复文件指针到文件头

    uint16_t map_name_len = 0;

    if (in.read(reinterpret_cast<char *>(&pose.x()), sizeof(pose.x()))
        && in.read(reinterpret_cast<char *>(&pose.y()), sizeof(pose.y()))
        && in.read(reinterpret_cast<char *>(&pose.yaw()), sizeof(pose.yaw()))
        && in.read(reinterpret_cast<char *>(&station_no), sizeof(station_no))
        && in.read(reinterpret_cast<char *>(&map_name_len), sizeof(map_name_len))) {
        char buffer[map_name_len];

        if (in.read(buffer, map_name_len)) {
            map_name = std::string(buffer, map_name_len);
            return true;
        }
    }

    return false;
}

void PoseSaver::oldSave(const sros::core::Pose &pose, const std::string &map_name, uint16_t station_no) {
    char buffer[32];

    std::ofstream out(OLD_SAVER_FILE_NAME, std::ifstream::out | std::ifstream::binary);

    if (out.fail()) {
        LOG(ERROR) << "failed to open PoseSaver file : " << OLD_SAVER_FILE_NAME;
        return;
    }

    strncpy(buffer, map_name.c_str(), 32);

    out.write(reinterpret_cast<const char *>(&pose.x()), sizeof(pose.x()));
    out.write(reinterpret_cast<const char *>(&pose.y()), sizeof(pose.y()));
    out.write(reinterpret_cast<const char *>(&pose.yaw()), sizeof(pose.yaw()));
    out.write(buffer, 32);
    out.write(reinterpret_cast<char *>(&station_no), sizeof(station_no));

    out.close();

    // 调用sync保证文件写入磁盘
    std::thread t([] { system("sync"); });
    t.detach(); // 脱离当前线程，执行完成后自动退出
}

bool PoseSaver::oldLoad(sros::core::Pose &pose, std::string &map_name, uint16_t &station_no) {
    char buffer[32];

    std::ifstream in(OLD_SAVER_FILE_NAME, std::ifstream::in | std::ifstream::binary);

    if (in.fail()) {
        LOG(ERROR) << "failed to open PoseSaver file : " << OLD_SAVER_FILE_NAME;
        return false;
    }

    // 判断文件大小是否合法
    in.seekg(0, std::ios_base::end);
    long size = in.tellg();

    if (size == 0) {
        LOG(ERROR) << "PoseSaver file is empty : " << OLD_SAVER_FILE_NAME;
        return false;
    }
    in.seekg(std::ios_base::beg); // 恢复文件指针到文件头

    in.read(reinterpret_cast<char *>(&pose.x()), sizeof(pose.x()));
    in.read(reinterpret_cast<char *>(&pose.y()), sizeof(pose.y()));
    in.read(reinterpret_cast<char *>(&pose.yaw()), sizeof(pose.yaw()));
    in.read(buffer, 32);

    station_no = 0;
    if (!in.eof()) {
        in.read(reinterpret_cast<char *>(&station_no), sizeof(station_no));
    }

    map_name = std::string(buffer);

    LOG(INFO) << "PoseSaver: " << pose.x() << ", " << pose.y() << "," << map_name << ", " << station_no;

    return true;
}
