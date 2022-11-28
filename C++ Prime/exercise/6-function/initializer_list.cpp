//
// Created by zxj on 2022/11/23.
//

#include <initializer_list>
#include <iostream>

int sum(std::initializer_list<int> l);
int main()
{
    std::cout << sum({1,2,3,4,5}) << std::endl;
}

int sum(std::initializer_list<int> l){
    int sum=0;
    for (auto beg=l.begin();beg!=l.end();beg++){
        sum+=*beg;
    }
    return sum;
}