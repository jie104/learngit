//
// Created by zxj on 2023/1/5.
//

#include<iostream>
#include <string>

int main()
{
#ifdef _CONSTRUCTOR_STRING_
    //std::string构造函数
    const char *cp="Hello World!!!";
    char noNull[]={'H','i'};
    std::string s1(cp); //拷贝cp中的字符直到遇到空字符；s1="Hello World!!!"
    std::string s2(noNull,2);   //从noNull拷贝两个字符；s2=="Hi"
    std::string s3(noNull);     //未定义，noNull不是以空字符结束
    std::string s4(cp+6,5); //从cp[6]开始拷贝5个字符；s4=="World"
    std::string s5(s1,6,5); //从s1[6]开始拷贝5个字符；s5=="World"
    std::string s6(s1,6);   //从s1[6]开始拷贝，直末尾；s7=="World!!!"
    std::string s7(s1,6,20);    //正确，只拷贝到s1末尾；s7=="World!!!"
    std::string s8(s1,16);  //抛出out_of_range异常

#endif

    //substr操作,返回一个string,它是原始string得一部分或全部得拷贝
    //可以传递给substr一个可选得开始位置和计数值
    std::string s("hello world");
    std::cout << "s: " << s << std::endl;
    std::string s2=s.substr(0,5);   //s2=hello
    std::cout << "s2: " << s2 << std::endl;
    std::string s3=s.substr(6);
    std::cout << "s3: " << s3 << std::endl;
    std::string s4=s.substr(6,11);
    std::cout << "s4: " << s4 << std::endl;
    //如果开始位置超过了string得大小，则substr函数抛出一个out_of_range异常
    std::string s5=s.substr(12);
    std::cout << "s5: " << s5 << std::endl;





}

