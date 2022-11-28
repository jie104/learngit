//
// Created by zxj on 2022/11/26.
//

#include <iostream>
#include <string>
typedef int Record;
typedef float Account;

//如果形参是某种类型的指针或引用，则通过区分其指向的是常量对象还是非常量对象可以实现函数重载
//此时的const是底层
Record lookup(Account&);
Record lookup(const Account&);

Record lookup(Account*);
Record lookup(const Account*);
int main()
{

}

const std::string &shortString(const std::string &s1,const std::string &s2)
{
    return s1.size() <=s2.size()? s1:s2;
}
//const_cast在重载函数的情境中最有用
std::string &shorterString(std::string &s1,std::string &s2){
    auto &r= shortString(const_cast<const std::string&>(s1),
                         const_cast<const std::string&>(s2));
    return const_cast<std::string&>(r);
}
