//
// Created by zxj on 2022/11/28.
//

#include <iostream>
void print(const int ia[],size_t size);

int main()
{
    int a[3]={1,2,3};
    print(a,3);

    const std::string word="sda";
    int threshold=5;
    if (word.size() <threshold ){
        std::cerr << "Error: " << __FILE__
                  << ": in function " << __func__
                  << " at line " << __LINE__ << std::endl
                  << "           Compiled on " << __DATE__
                  << " at " << __TIME__ << std::endl
                  << "            Word read was \"" << word
                  << "\": Length too short " << std::endl;
    }
}
void print(const int ia[],size_t size)
{
#ifndef NDEBUG
    //__func__是编译器定义的一个局部静态变量，用于存放函数的名字
    std::cerr << __func__ << ": array size is " << size << std::endl;
#endif
}


