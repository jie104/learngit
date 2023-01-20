//
// Created by zxj on 2023/1/10.
//

#include <iostream>
#include <numeric>
#include <vector>
#include <list>

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
    //fill_n接受一个单迭代器、一个计数值和一个值，它将给定值赋予迭代器指向的元素开始的指定个迭代器
    //赋值意味着容器中应该有对应元素个数才可以赋值
    std::vector<int> vec1;
//    vec1.reserve(10);
    vec1.resize(10);
    std::fill_n(vec1.begin(),10,0);
    for (auto &x:vec1){
        std::cout << "vec1: " << x << std::endl;
    }

}