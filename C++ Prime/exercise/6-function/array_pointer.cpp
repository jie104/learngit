//
// Created by zxj on 2022/11/26.
//

#include <iostream>
#include <string>

//类型别名
typedef std::string s[10];
s& f1(int i);

//函数声明
std::string (&f(int i))[10];

//尾置返回
auto f2(int i) ->std::string (&)[10];

//使用decltype
std::string s1[10];
decltype(s1)& f3(int i);

int odd[]={1,3,5,7,9};
int even[]={0,2,4,6,8};
int main()
{

}

decltype(odd)& arrPtr(int i){
    return (i%2) ? odd:even;
}