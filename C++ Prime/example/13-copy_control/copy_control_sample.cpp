//
// Created by zxj on 2023/3/27.
//

#include <iostream>
#include <vector>
#include <string>
#include <set>

class Folder{

};

///Message类
class Message{
    friend class Folder;
public:
    //folders被隐式初始化为空集合
    explicit Message(const std::string &str=" "):
        contents(str) {}
    //拷贝控制成员，用来管理指向本Message的指针
    Message(const Message&);
    Message& operator=(const Message&);
    ~Message();
    //从给定Folder集合中添加/删除本Message
    void save(Folder&);
    void remove(Folder&);

private:
    std::string contents;   //实际消息文本
    std::set<Folder*> folders;  //包含本Message的Folder
    //拷贝构造函数、拷贝赋值运算符和析构函数所使用的工具函数
    //将本Message添加到指向参数的Folder
    void add_to_Folders(const Message&);
    //从folders中每个Folder中删除本Message
    void remove_from_Folders();
};
