//
// Created by zxj on 2022/11/16.
//
#include <iostream>

size_t count_calls()
{
    static size_t ctr=0;    //调用结束后，这个值仍然有效
    return ++ctr;
}

int main()
{
    for (size_t i=0;i!=10;++i){
        std::cout << count_calls() << std::endl;
    }
    return 0;
}
