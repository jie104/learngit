//
// Created by zxj on 2023/1/9.
//
#include <iostream>
#include <numeric>
#include <vector>
#include <string>
#include <set>
#include<iterator>
#include <algorithm>

//标准库算法不能直接对迭代器而不是容器进行操作。因此，算法不能直接添加或删除元素
void elimDups(std::vector<std::string> &words)
{
    //按字典排列words,以便查找单词
    std::sort(words.begin(),words.end());
    //unique重排输入范围ie，使得每个元素只出现一次
    //排列在范围的前部，返回不重复区域之后一个位置的迭代器
    auto end_unique=std::unique(words.begin(),words.end());

    //使用向量操作erase删除重复单词
    words.erase(end_unique,words.end());
}

bool isShorter(const std::string &s1,const std::string &s2){
    return s1.size()<s2.size();
}

int main()
{
//    std::vector<int> vec={0,1,2,3,4,5,6,7,8,9};
//    //对vec中的元素求和，和初始值是0
//    //accmulate的第三个参数决定了函数中使用哪个加法运算符以及返回值的类型
//    int sum=std::accumulate(vec.begin(),vec.end(),0);
//    std::cout << "sum: "<< sum << std::endl;
//
//    std::vector<std::string> s={"dasfaf","caa","rryw"};
//    std::string sum1=std::accumulate(s.cbegin(),s.cend(),std::string(""));
//    std::cout << "s: " << sum1 << std::endl;

    std::vector<int> ivec={2,4,6,8,2,4,6,8};
    std::set<int> set2;
    set2.insert(ivec.cbegin(),ivec.cend());
    set2.insert({1,3,5,7,1,3,5,7});
    for (auto &x:set2){
        std::cout << x << std::endl;
    }

//equal
    std::vector<int> roster1={1,2,3,3,4,5,6,7,8};
    std::vector<int> roster2={1,2,3,4,5,5,6,7,8};
    bool flag=std::equal(roster1.begin(),roster1.end(),roster2.begin());
    if (flag){
        std::cout << "roster1==roster2" << std::endl;
    }else{
        std::cout << "roster1!=roster2" << std::endl;
    }

//fill
    //将元素每个元素重置为0
    std::fill(ivec.begin(),ivec.end(),0);
    for (auto &x:ivec){
        std::cout << "ivec: " << x << std::endl;
    }
    //将容器一个子序列设置为10
    std::fill(ivec.begin(),ivec.begin()+ivec.size()/2,10);
    for (auto &x:ivec){
        std::cout << "ivec1: " << x << std::endl;
    }

//fill_n,接受一个单迭代器，一个计数器，一个值
    //它将给定值赋予迭代器指向的元素开始的指定元素
    std::vector<int> vec={1,2,3,4};
    std::fill_n(vec.begin(),vec.size(),0);  //将所有元素重置为0
    for (auto &x: vec){
        std::cout << "vec: " << x << std::endl;
    }

    //指定要写入10个元素，单vec中并没有元素，它是空，未定义
//    std::fill_n(vec.begin(),10,0);

//back_inserter,在头文件iterator中
    //back_inserter接受一个指向容器的引用，返回一个与该容器绑定的插入迭代器
    //当通过此迭代器赋值时，赋值运算符会调用push_back将一个具有给定值的元素添加到容器中
    std::vector<int> vec2;
    auto it=std::back_inserter(vec2);   //通过它赋值会将元素添加到vec2中
    *it=42;
    *it=34;
    *it=56;
    for (auto &x: vec2){
        std::cout << x << std::endl;
    }

    //常常使用back_inserter来创建一个迭代器，作为算法的，目地位置使用
    std::vector<int> vec3;
    std::fill_n(std::back_inserter(vec3),10,0);
    for (auto &x: vec3){
        std::cout << "vec3: " << x << std::endl;
    }

    //拷贝算法copy
    int a1[]={0,1,2,3,4,5,6,7,8,9};
    int a2[sizeof(a1)/sizeof(*a1)];
    //ret指向拷贝到a2的尾元素之后的位置
    auto ret=std::copy(std::begin(a1),std::end(a1),a2); //将a1内容拷贝到a2

//重排容器元素的算法
    //sort
    std::vector<std::string> words={"the","the","over","jump","quick","turtle","fox"};
    //按字典序排序words
//    std::sort(words.begin(),words.end());
//    for (auto &x:words){
//        std::cout << "words: " << x << std::endl;
//    }

//谓词是一个可调用的表达式，其返回结果是一个能用作条件的值
//    std::sort(words.begin(),words.end(), isShorter);
//    for (auto &x:words){
//        std::cout << "words_: " << x << std::endl;
//    }

    //按谓词顺序排序,并把等长度的字母保持字典序，默认情况下按字典序排序
    std::stable_sort(words.begin(),words.end(), isShorter);
    for (auto &x:words){
        std::cout << "words1: " << x << std::endl;
    }
}


