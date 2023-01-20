//
// Created by zxj on 2023/1/10.
//

#include <iostream>
#include <numeric>
#include <vector>
#include <list>
#include <algorithm>
//#define _TEN_FOURTEEN_


void biggies(std::vector<std::string> &words,
             std::vector<std::string>::size_type sz,
             std::ostream &os=std::cout,char c=' '){
    //os隐式捕获，引用捕获方式；c显式捕获，值捕获方式
    std::for_each(words.begin(),words.end(),
                  [&,c](const std::string &s){os << s << c;});
    //os显式捕获，引用捕获方式；c隐式捕获，值捕获方式
    std::for_each(words.begin(),words.end(),
                  [=,&os](const std::string &s) {os << s << c;});
}

//值捕获
void fcn1()
{
    size_t v1=42;   //局部变量
    //将v1拷贝到名为f的可调用对象
    auto f=[v1]{return v1;};
    v1=0;
    auto j=f(); //j为42；f保存了我们创建它时v1的拷贝
}

//引用捕获
//采用引用方式捕获变量
void fcn2()
{
    size_t v1=42;   //局部变量
    //对象f2包含v1的引用
    auto f2=[&v1]{return v1;};
    v1=0;
    auto j=f2();    //j为0；f2保存v1的引用，而非拷贝
}


int main()
{
#ifdef _TEN_THREE_
    //exercise 10.3
    std::vector<int> v={0,1,2,3,4,4,5,56,6,6,7};
    auto sum=std::accumulate(v.begin(),v.end(),0);
    std::cout << "sum: " << sum << std::endl;

#endif

#ifdef _TEN_FOUR_
    //exercise 10.4
    std::vector<double> v1={0,1,2,3.5,4,4,5,5.6,6,6.3,7};
    auto sum1=std::accumulate(v1.begin(),v1.end(),0.0);
    std::cout << "sum1: " << sum1 << std::endl;
#endif

#ifdef _TEN_FIVE_
    //exercise 10.5
    std::vector<std::string> roster1={"xdasdffa","dafdqa","cxzxvxvz"};
    std::list<const char*> roster2={"xdasdffa","dafdqa","cxzxvxvz"};
    bool flag=std::equal(roster1.begin(),roster1.end(),roster2.begin());
    if (flag){
        std::cout << "roster1==roster2" << std::endl;

    }else{
        std::cout << "roster1!=roster2" << std::endl;
    }
#endif

#ifdef _TEN_SIX_
    //exercise 10.6
    std::vector<int> v={1,2,3,3,4,5,6,7,8};
    std::fill_n(v.begin(),v.size(),0);
    for (auto &x:v){
        std::cout << x << std::endl;
    }
#endif

#ifdef _TEN_SEVEN_
    //exercise 10.7
    std::vector<int> vec;
    std::list<int> lst;
    int i;
    while (std::cin >> i && i!=1){
        lst.push_back(i);
    }
    vec.resize(lst.size());
    //传递给copy的目地序列至少要包含与输入序列一样多的元素
    std::copy(lst.cbegin(),lst.cend(),vec.begin());
    for (auto &x:vec){
        std::cout << "vec: " << x << std::endl;
    }

#endif
/*
    //fill_n接受一个单迭代器、一个计数值和一个值，它将给定值赋予迭代器指向的元素开始的指定个迭代器
    //赋值意味着容器中应该有对应元素个数才可以赋值
    std::vector<int> vec1;
//    vec1.reserve(10);
    vec1.resize(10);
    std::fill_n(vec1.begin(),10,0);
    for (auto &x:vec1){
        std::cout << "vec1: " << x << std::endl;
    }
*/
    //exercise 10.14
#ifdef _TEN_FOURTEEN_
    int a=1,b=4;
    auto sum=[](int &a,int &b){return a+b;};
    std::cout << sum(a,b) << std::endl;
#endif

#ifdef _TEN_FIFTEEN_
    //exercise 10.15
    //一个lambda只有在其捕获列表中捕获一个它所在函数中的局部变量，才能在函数体中使用该变量
    int a=1;
    auto f=[a](int b){return a+b;};
    std::cout << f(5) << std::endl;


    //sz为隐式捕获，值捕获方式
    int sz=4;
    std::vector<std::string> words={"dsada","twew","vbbcc"};
    auto wc=std::find_if(words.begin(),words.end(),
                         [=](const std::string &s){return s.size()>=sz;});
#endif

    //exercise 10.20
    std::vector<std::string> words={"qweqgds","efw","bxcvxrtew","czqa"};
    auto num=std::count_if(words.begin(),words.end(),
                  [](std::string word){return word.size()>6?true: false;});
    std::cout << "num: " << num << std::endl;

    //exercise 10.21
    int k=32;
    auto flag=[k]() mutable ->bool{
        if (k==0){
            std::cout << "k is zero" << std::endl;
            return false;
        }
        while (k!=0){
            --k;
        }
        std::cout << "k is not zero" << std::endl;
        return true;
    };
    std::cout << flag() << std::endl;
}