
#include <fstream>
#include <iostream>

#include <boost/algorithm/string.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include "parse.h"

using std::ofstream;
using namespace std;

bool write_file(const std::string &file_path, const std::string &file_content) {
    if (file_path == "" || file_content == "") {
        std::cout << "write_file(): file_path=" << file_path << " file_content=" << file_content << std::endl;
        return false;
    }
    ofstream out(file_path, ifstream::out | ifstream::binary);
    if (out.fail()) {
        cerr << "Failure to open for file " << file_path << endl;
        return false;
    }

    out << file_content;
    out.close();
    return true;
}

string get_create_json_file_path(const string &monitor_file) {
    boost::filesystem::path mon_file_path(monitor_file);
    string base_name = mon_file_path.stem().string();
    string file_path = boost::filesystem::system_complete(mon_file_path).parent_path().string();
    return file_path + "/" + base_name + ".json";
}

void get_split_files(const string &input, vector<string> &file_list) {
   boost::split(file_list, input, boost::is_any_of(","));
}

void displayCurrentTime () {
    const boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    // Get the time offset in current day
    const boost::posix_time::time_duration td = now.time_of_day();
    uint16_t ms = td.total_milliseconds();
    std::cout << "current time " << ms << std::endl;
}


int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./sros_monitor_file_to_json monitor_file.mf" << std::endl;
        exit(-1);
    }

    std::string input = argv[1];
    vector<string> files;
    get_split_files(input, files);
    Parse file_parser;

    for (uint32_t i = 0; i < files.size(); i++) {

//        std::cout << "start parse file time: " << std::endl;
//        displayCurrentTime();

        string monitor_file = files[i];
        std::string str_json;
        if (!file_parser.parseMonitorFile(monitor_file, str_json)) {
            continue;
        }

//        std::cout << "finish parse file time: " << std::endl;
//        displayCurrentTime();

        std::string json_file_path = get_create_json_file_path(monitor_file);
        if (!write_file(json_file_path, str_json)) {
            continue;
        }

//        std::cout << "write file use time: " << std::endl;
//        displayCurrentTime();
    }

    return 0;
}

