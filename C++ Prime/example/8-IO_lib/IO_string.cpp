//
// Created by zxj on 2022/12/25.
//

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

struct PersonInfo{
    std::string name;
    std::vector<std::string> phones;
};

int main()
{
    std::string s;
    //strm是一个未绑定的stringstream对象
    std::stringstream strm;
    //strm是一个sstream对象，保存string s的一个拷贝，此构造函数是explicit
    std::stringstream strm1(s);
    //返回strm所保存的string的拷贝
    strm.str();
    //将string s拷贝到strm中，返回void
    strm.str(s);

    std::string line,word;
    std::vector<PersonInfo> people;
    //将istringstream定义放到外层的解决方法
    std::istringstream record;

    //逐行从输入读取数据，知道cin遇到文件尾或其他错误
    while (getline(std::cin,line) && line!="q"){
        PersonInfo info;
        record.clear();
        record.str(line);
        record >> info.name;
        while (record >> word)
            info.phones.push_back(word);
        people.push_back(info);
    }
    for (auto& x: people){
        std::cout << "name:" << x.name << std::endl;
        for (auto& y:x.phones){
            std::cout << "phone:  " << y << std::endl;
        }
    }

    for (const auto &entry:people){
        std::ostringstream formatted,badNums;
        for(const auto &nums:entry.phones){
            if(!valid(nums)){
                badNums << " " << nums;
            }else
                formatted << " " << format(nums);
        }
        if (badNums.str().empty())
            os << entry.name << " "
               << formatted.str() << std::endl;
        else
            std::cerr << "input error: " << entry.name
                      << " invalid number(s) " << badNums.str() << std::endl;
    }
}


