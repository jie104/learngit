//
// Created by lfc on 2021/9/17.
//

#ifndef SROS_RECORD_FILE_MANAGER_HPP
#define SROS_RECORD_FILE_MANAGER_HPP
#include <map>
namespace record{
class RecordFileManager {
 public:
    static std::string creatImgName(tm* curr_time){
        std::stringstream file_name;
        file_name << (curr_time->tm_year + 1900) << "-" << (curr_time->tm_mon + 1) << "-" << curr_time->tm_mday << "-"
                  << curr_time->tm_hour << "-" << curr_time->tm_min << "-" << curr_time->tm_sec<<".";
        file_name << curr_time->tm_yday << ".day";
        LOG(INFO) << "file name is:" << file_name.str();
        return file_name.str();
    }

    static int64_t convertToTimeInMinutes(const std::string time_str){
        std::stringstream time_stream;
        time_stream << time_str;
        std::string time_line;
        int64_t real_time = 0;
        std::getline(time_stream,time_line,'-');//年度
        std::getline(time_stream,time_line,'-');//月度
        real_time = deltaDate(0, time_line);
        std::getline(time_stream,time_line,'-');//日度
        real_time = real_time * 31 + deltaDate(0, time_line);
        std::getline(time_stream,time_line,'-');//时度
        real_time = real_time * 24 + deltaDate(0, time_line);
        std::getline(time_stream,time_line,'-');//分度
        real_time = real_time * 60 + deltaDate(0, time_line);
        return real_time;
    }

    static struct tm* getCurrSaveTime(){
        auto curr_time_stamp = std::time(nullptr);
        auto curr_time = std::localtime(&curr_time_stamp);
        return curr_time;
    }

    static double timeinus(){
        struct timeval tv;
        struct timezone tz;
        gettimeofday(&tv, &tz);
        return tv.tv_usec;
    }

    static int deltaDate(const int &curr_date,std::string &last_date){
        try {
            int last_date_int = std::stoi(last_date);
            auto delta_time = curr_date - last_date_int;
            return abs(delta_time);
        } catch (...) {
            LOG(INFO) << "cannot convert date:" << last_date;
            return 100;
        }
    }

    static void manageRecordFile(std::string path,std::string suffix,int day_long){
        static std::map<std::string,bool> have_managed_map;
        if (have_managed_map.find(suffix) == have_managed_map.end()) {
            LOG(INFO) << "will update suffix!" << suffix;
            have_managed_map[suffix] = true;
        }else{
            LOG(INFO) << "will not update the file info:" << suffix;
            return;
        }
        if(!boost::filesystem::exists(path)){
            boost::filesystem::create_directory(path);
            return;
        }
        boost::filesystem::directory_iterator begin(path);
        boost::filesystem::directory_iterator end;
        std::vector<std::string> remove_files;
        std::vector<std::pair<int64_t,std::string>> all_files;
        auto curr_real_time = getCurrSaveTime();
        for (;begin != end; begin++) {
            if (!boost::filesystem::is_directory(*begin)) {
                std::string file_name = begin->path().filename().string();
                LOG(INFO) << "file name is:" << file_name;
                if (file_name.empty()) {
                    LOG(INFO) << "file is empty!";
                    continue;
                }
                auto curr_position = file_name.find(suffix);//确定是png
                if (curr_position > 0 && curr_position < file_name.size()) {
                    std::stringstream file_stream;
                    file_stream << file_name;
                    std::string curr_time,curr_date;
                    std::getline(file_stream, curr_time, '.');
                    std::getline(file_stream, curr_date, '.');
                    auto delta_date = deltaDate(curr_real_time->tm_yday, curr_date);
                    if (delta_date > day_long) {//如果大于2天，数据自动清除
                        remove_files.push_back(begin->path().string());
                    }else{
                        all_files.emplace_back(convertToTimeInMinutes(curr_time),begin->path().string());
                    }

                    LOG(INFO) << "file name:" << file_name << "," << delta_date;
                }
            }

        }
        for (auto &file : remove_files) {
            boost::filesystem::remove(file);
        }
        if (all_files.size() > 128) {
            LOG(INFO) << "too many! will remove part file!";
            std::sort(all_files.begin(),all_files.end(),
                [](const std::pair<int64_t,std::string>& a,const std::pair<int64_t,std::string>& b){return a.first > b.first;
                      });
            int incre = 0;
            for(auto& file:all_files){
                if (incre++ > 128) {
                    boost::filesystem::remove(file.second);
                }
            }
        }
    }
 private:

};
}


#endif  // SROS_RECORD_FILE_MANAGER_HPP
