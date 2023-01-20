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
#include <functional>

std::ostream &print(std::ostream &os,const std::string &s,char c){
    return os << s << c;
}

//此声明说明名字_1定义在名字空间placeholders中，而命名空间又定义在std中
using std::placeholders::_1;

//说明来自std::placeholders的名字可以在我们程序中直接使用
using namespace std::placeholders;

double f(double a,double b,double c,double d,double f){
    return a+b+c+d-f;
}


bool check_size(const std::string &s,std::string::size_type sz){
    return s.size() >= sz;
}

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

/***
    可以将bind看作一个通用的函数适配器，它接受一个可调用对象，生成一个新的可调用对象来
    “适应”原对象的参数列表
    调用bind的一般形式：
    auto newCallable=bind(callable,arg_list);
    其中，newCallable本身是一个可调用对象，arg_list是一个逗号分隔的参数列表
 ***/
    auto newCallable=std::bind(isShorter,"sdafa","gwgw");
    std::cout << "bind: " << newCallable() << std::endl;

    auto check6=std::bind(check_size,std::placeholders::_1,6);
    std::string s="hello";
    bool b1=check6(s);  //check6(s) 会调用check_size(s,6)

/***
    传递给g的参数按位置绑定到占位符，即第一个参数绑定到_1,第二个参数绑定到_2
    auto g=bind(f,a,b,_2,c,_1)
    bind调用会将
        g(_1,_2)
    映射为
        f(a,b,_2,c,_1)
 ***/
    auto g=std::bind(f,0,0,_2,0,_1);
    std::cout << "g: " << g(3,2) << std::endl;

    //用bind重排参数顺序
    std::cout << "正常比较顺序.................." << std::endl;
    std::sort(words.begin(),words.end(), isShorter);
    for(auto &x:words){
        std::cout << x << std::endl;
    }

    //按单词长度由长至短排列
    std::cout << "比较顺序交换..................." << std::endl;
    std::sort(words.begin(),words.end(),std::bind(isShorter,_2,_1));
    for(auto &x:words){
        std::cout << x << std::endl;
    }

    //ostream不能被拷贝
    std::ostream &os=std::cout;
    std::for_each(words.begin(),words.end(),
                  std::bind(print,std::ref(os),_1,' '));
}

