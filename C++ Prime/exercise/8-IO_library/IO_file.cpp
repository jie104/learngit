//
// Created by zxj on 2022/12/24.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

std::vector<std::string> read(std::ifstream& file_name){
    std::string line;
    std::string word;
    std::vector<std::string> v;
    //将每一行作为独立的元素存于vector
//    while (std::getline(file_name,line)){
//        v.emplace_back(line);
//    }
    //将每个单词作为独立的元素进行存储
    while (file_name >> word){
        if (file_name){
            v.emplace_back(word);
        }else{
            std::cout << "the file input error" << std::endl;
        }
    }
    return v;
}

int main()
{
    std::ifstream in("/home/zxj/file.txt");
    std::vector<std::string> v=read(in);
//    std::cout << "size: " << v.size();
    for (auto &x:v){
        std::cout << x << std::endl;
    }

    //以out模式打开ia文件会丢失已有数据
    std::ofstream out("file1"); //隐含以输出模式打开文件并截断文件
    std::ofstream out2("file1",std::ofstream::out); //隐含截断文件
    std::ofstream out3("file1",std::ofstream::out | std::ofstream::trunc);
    //为了保留文件内容，必须显式指定app模式
    std::ofstream app("file2" ,std::ofstream::app); //隐含输出模式
    std::ofstream app2("file2",std::ofstream::out | std::ofstream::app);
}
