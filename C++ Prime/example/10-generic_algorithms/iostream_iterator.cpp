//
// Created by zxj on 2023/1/30.
//

#include <iostream>
#include <iterator>
#include <vector>
#include <algorithm>
#include <numeric>
#include <memory>


struct school
{
    int number=1000;
    int boy_num=400;
    int girl_num=600;

    struct classroom{
        int desk_num=20;
    };
};


int main()
{
#ifdef _CIN_
    std::vector<int> vec;
    std::istream_iterator<int> in_iter(std::cin);//从cin读取int
    std::istream_iterator<int> eof;

    /***
     对于一个绑定到流的迭代器，一旦其关联的流遇到文件尾或遇到IO错误，
     迭代器的值就与尾后迭代器相等
    ***/
    std::cout <<  "*****************: " << 1 << std::endl;
     while (in_iter!=eof){
         //输入流迭代器每使用一次后置运算符就会从流中读取下一个值
        vec.push_back(*in_iter++);
    }

    for(auto &x:vec){
        std::cout << "vec: " << x << std::endl;
    }

//效果与上面无异
    //可以将程序重写为如下形式，体现了istream_iterator更有用的地方
    std::istream_iterator<int> in_iter1(std::cin),eof1;
    std::vector<int> vec1(in_iter1,eof1);    //从迭代器范围构造vec

    for(auto &x:vec1){
        std::cout << "vec1: " << x << std::endl;
    }

#endif
    //使用算法操作流迭代器
    std::istream_iterator<int> in(std::cin),eof2;
    std::cout << std::accumulate(in,eof2,0) << std::endl;

    std::shared_ptr<school> school_ptr;
    school_ptr.reset(new school);
    std::cout << "school_ptr_num: " << school_ptr->boy_num << std::endl;


}