//
// Created by zxj on 2022/11/25.
//

#include <iostream>
#include <vector>
//arrT是一个类型别名，它表示的类型是含有10个整数的数组
typedef int arrT[10];
int odd[] ={1,3,5,7,9};
int even[]={0,2,4,6,8};

//一个拥有顶层const的形参无法与另一个没有顶层的形参区分开来
int lookup(int i);
int lookup(const int i);

int main()
{
    //arrT是一个类型别名，它表示的类型是含有10个整数的数组
    using arrT1=int[10];

    int arr[10];    //arr是一个含有10个整数的数组
    int *p1[10];    //p1是一个含有10个指针的数组
    int (*p2)[10] =&arr;    //p2是一个指针，它指向含有10个整数的指针

    int p[2]={1,2};
    auto beg=std::begin(p);

    std::vector<int> v={1,2,3,4};
    std::vector<int>::iterator beg1=v.begin();
    auto end1=v.end();

    lookup(1);

    //定位四个重载函数

}
int lookup(int i){
    return i;
}

//func返回一个指向含有10个整数的数组的指针
arrT* func1(int i){

}

//使用尾置返回类型
auto func2(int i) ->int(*)[10]{

}
//声明一个返回数组指针的函数
int (*func3(int i))[10]{

}

decltype(odd) *arrPtr(int i)
{
    return (i%2) ? &odd: &even; //返回一个指向数组的指针
}