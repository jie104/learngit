#include <iostream>
#include <string>
#include <vector>

int main()
{
#ifdef _INBUILT_ARRAY_1_
    constexpr size_t array_size=10;
    int ia[array_size];
    for (size_t ix=0;ix <array_size;++ix){
        ia[ix]=ix;
    }

    for (size_t j=0;j< array_size;++j){
        ia[j]=j;
    }

    std::string nums[]={"one","two","three"};
    std::string *p=&nums[0];
//使用数组类型的对象其实是使用一个指向该数组首元素的指针
    std::string *p2=nums;   //等价于p2=&nums[0]


    int ia[]={0,1,2,3,4,5,6,7,8,9}; //ia是一个含有10个整数的数组
//编译器实际执行的初始化过程类似 auto ia2(&ia[0])
    auto ia2(ia);   //ia2是一个整数指针，指向ia的第一个元素

//ia3是一个含有10个整数的数组
    decltype(ia) ia3={0,1,2,3,4,5,6,7,8,9};
    auto ia5(ia3);
    // ia3=p;  //错误：不能用整型指针给数组赋值
    ia3[4]=5;

    int ia4[10]={0,1,2,3,4,5,6,7,8,9};

    int *e=&ia4[10];
    for (int *b=ia4;b!=e;++b){
        std::cout << *b << std::endl;
    }
#endif

    int ia[]={0,1,2,3,4,5,6,7,8,9}; //ia是一个含有10个整数的数组

    int *beg=std::begin(ia);    //指向ia首元素的指针
    int *last=std::end(ia); //指向ia尾元素的下一位置的指针

    while (beg!=last && *beg >=0){
        ++beg;
    }
} 