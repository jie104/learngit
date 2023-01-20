//
// Created by zxj on 2022/12/25.
//

#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <fstream>

std::vector<std::string> read(std::ifstream& file_name){
    std::string line;
    std::string word;
    std::vector<std::string> v;
    //将每一行作为独立的元素存于vector
    while (std::getline(file_name,line)){
        v.emplace_back(line);
    }
    //将每个单词作为独立的元素进行存储
//    while (file_name >> word){
//        if (file_name){
//            v.emplace_back(word);
//        }else{
//            std::cout << "the file input error" << std::endl;
//        }
//    }
    return v;
}

int main()
{
    //exercise8.10
    std::ifstream in("/home/zxj/file.txt");
    std::vector<std::string> v;
    v=read(in);

    for (auto &x:v){
        std::istringstream is(x);
    }
}
