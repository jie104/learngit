////
//// Created by lfc on 17-1-5.
////
//
//#include <glog/logging.h>
//#include "pose_record.h"
//
//#define name_to_str(value_name) #value_name
//
//location::RecordDebugInfo::RecordDebugInfo() : is_open(false) {
//
//}
//
//record::PoseRecord::~PoseRecord() {
//    is_open = false;
//    record_stream.close();
//}
//
//bool record::PoseRecord::openFile(const char *path) {
//    record_stream.open(path, std::ios::out);
//    if (record_stream.is_open()) {
//        is_open = true;
//        return true;
//    }
//    record_stream.precision(12);
//    LOG(INFO) << "cannot open the file!";
//    is_open = false;
//    return false;
//}
//
//void record::PoseRecord::writePose(Eigen::Vector3f &pose, std::string name) {
//    std::string tmp_name = name + "_x";
//    writeData(pose.x(), tmp_name);
//    tmp_name = name + "_y";
//    writeData(pose.y(), tmp_name);
//    tmp_name = name + "_yaw";
//    writeData(pose.z(), tmp_name);
//}
//
//void record::PoseRecord::writeCov(Eigen::Matrix3f &cov, std::string name) {
//    std::string tmp_name = name + "_00";
//    writeData(cov(0, 0), tmp_name);
//    tmp_name = name + "_01";
//    writeData(cov(0, 1), tmp_name);
//    tmp_name = name + "_11";
//    writeData(cov(1, 1), tmp_name);
//    tmp_name = name + "_02";
//    writeData(cov(0, 2), tmp_name);
//    tmp_name = name + "_12";
//    writeData(cov(1, 2), tmp_name);
//    tmp_name = name + "_22";
//    writeData(cov(2, 3), tmp_name);
//}
//
////void record::PoseRecord::writeRecordItem(record::RecordItem_ptr record_ptr) {
////    std::string write_name = name_to_str(record_ptr->scan_stamp);
////    writeData(record_ptr->scan_stamp, selectStrByArrow(write_name));
////    write_name = name_to_str(record_ptr->locating_state);
////    writeData(record_ptr->locating_state, selectStrByArrow(write_name));
////    write_name = name_to_str(record_ptr->pose_output_type);
////    writeData(record_ptr->pose_output_type, selectStrByArrow(write_name));
////    write_name = name_to_str(record_ptr->pose_chain_size);
////    writeData(record_ptr->pose_chain_size, selectStrByArrow(write_name));
////    write_name = name_to_str(record_ptr->pose_error_chain_size);
////    writeData(record_ptr->pose_error_chain_size, selectStrByArrow(write_name));
////    write_name = name_to_str(record_ptr->pitch);
////    writeData(record_ptr->pitch, selectStrByArrow(write_name));
////    write_name = name_to_str(record_ptr->roll);
////    writeData(record_ptr->roll, selectStrByArrow(write_name));
////    write_name = name_to_str(record_ptr->pose_map_initialpose);
////    writePose(record_ptr->pose_map_initialpose, selectStrByArrow(write_name));
////    write_name = name_to_str(record_ptr->pose_odo);
////    writePose(record_ptr->pose_odo.pose, selectStrByArrow(write_name));
////    write_name = name_to_str(record_ptr->pose_output);
////    writePoseInfo(record_ptr->pose_output, selectStrByArrow(write_name), false);
////    write_name = name_to_str(record_ptr->pose_rt);
////    writePoseInfo(record_ptr->pose_rt, selectStrByArrow(write_name), false);
////    //TODO:没有加ukf的位姿
////    int map_size = record_ptr->pose_maps.size();
////    for (int i = 0; i < map_size; ++i) {
////        std::stringstream name_stream;
////        write_name = name_to_str(record_ptr->pose_maps);
////        name_stream << write_name << "_" << i;
////        write_name = name_stream.str();
////        writePoseInfo(record_ptr->pose_maps[i], selectStrByArrow(write_name));
////    }
////
////    writeEndl();
////}
//
//void record::PoseRecord::close() {
//    is_open = false;
//    record_stream.close();
//}
//
//std::string record::PoseRecord::selectStrByArrow(std::string input) {
//    std::string output = input.substr(input.find("->", 0) + 2);
//    return output;
//}
//
//
//std::string record::PoseRecord::selectStrBydot(std::string input) {
//    std::string output = input.substr(input.find(".", 0) + 1);
//    return output;
//}
//
//void record::PoseRecord::writeEndl() {
//    if (is_open) {
//        record_stream << std::endl;
//    }
//}
