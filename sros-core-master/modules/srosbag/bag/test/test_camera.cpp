//
// Created by duan on 2021/6/16.
//

#include "bag/message_bag.h"
#include "factory/my_factory.h"
#include <array>
#include <dirent.h>
#include <functional>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <typeinfo>
#include <vector>

std::vector<std::string> getFiles(std::string cate_dir)
{

    std::vector<std::string> files; //存放文件名

    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if ((dir = opendir(cate_dir.c_str())) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr = readdir(dir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) ///current dir OR parrent dir
            continue;
        else if (ptr->d_type == 8) ///file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            files.push_back(ptr->d_name);
        else if (ptr->d_type == 10) ///link file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            continue;
        else if (ptr->d_type == 4)
        { ///dir
            files.push_back(ptr->d_name);
        }
    }
    closedir(dir);

    //排序，按从小到大排序
    std::sort(files.begin(), files.end());
    return files;
}

int main() {
    auto bag = bag::MsgBag::Create("/home/duan/message_bag");
    std::vector<std::string> names;
    names.emplace_back("TOPIC_D435_COLOR");
//    names.emplace_back("right_cam");
    names.emplace_back("TOPIC_D435_DEPTH");
     bag->startRecord(names);

    std::string rgb_name = "/home/duan/myWorkStation/dataset/TUM/handheld/fr1_desk/rgb/";
    std::string depth_name = "/home/duan/myWorkStation/dataset/TUM/handheld/fr1_desk/depth/";
    std::vector<std::string> rgb_names = getFiles(rgb_name);
    std::vector<std::string> depth_names = getFiles(depth_name);
    LOG(INFO) << "rgb names size is :" << rgb_names.size();
    LOG(INFO) << "rgb name is :" << rgb_names[0];

//    for (int i = 0; i < rgb_names.size(); i++) {
    for (int i = 0; i < 5; i++) {
        //         cv::Mat rgb = cv::imread("/home/zmy/Pictures/Selection_008.png");
        cv::Mat rgb = cv::imread(rgb_name + rgb_names[i]);
        cv::Mat depth = cv::imread(depth_name + depth_names[i], CV_LOAD_IMAGE_UNCHANGED);

        sros::core::ImageMsg left_cam("left_cam");
        sros::core::ImageMsg right_cam("right_cam");
        sros::core::ImageMsg depth_cam("depth_cam");

        left_cam.time_ = sros::core::util::get_timestamp_in_us();
        left_cam.mat_ = rgb.clone();

        depth_cam.time_ = left_cam.time_;
        depth_cam.mat_ = depth.clone();
        //
        bag->dumpMsg<sros::core::ImageMsg>(left_cam, "TOPIC_D435_COLOR");
//        bag->dumpMsg<sros::core::ImageMsg>(right_cam, "TOPIC_D435_COLOR");
        bag->dumpMsg<sros::core::ImageMsg>(depth_cam,"TOPIC_D435_DEPTH");
    }
//    }

    bag->stopRecord();
//
//    bag->setMsgHandle<sros::core::ImageMsg>(
//        [](const sros::core::ImageMsg &camera) {
//          std::cout << "camera stamp:" << camera.time_ << std::endl;
//          std::cout << "left image size is :" << camera.mat_.size() << std::endl;
//
//          cv::namedWindow("left", cv::WINDOW_NORMAL);
//          if (!camera.mat_.empty()) cv::imshow("left", camera.mat_);
//          cv::waitKey(5);
//        },
//        "left_cam");
//
//    bag->playBack("2021-06-17-19-53-52.bag", true);

    return 0;
}