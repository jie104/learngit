//
// Created by zxj on 2022/11/10.
//

#include <iostream>
#include <vector>

int main()
{
    std::vector<int> v;
    auto beg=v.begin();
    for (;beg!=v.end() && *beg >=0;++beg){
        ;
    }
    //省略condition的效果等价于在条件部分写了一个true
    for (int i=0;;++i){

    }

    std::vector<int> v={0,1,2,3,4,5,6,7,8,9};
    for (auto &r:v)
        r=r*2;
}