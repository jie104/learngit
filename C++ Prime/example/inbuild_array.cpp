#include <iostream>
#include <vector>
#include <string>
constexpr int f(int x){
    return x;
}

int main()
{
    unsigned cnt=42;
    constexpr unsigned sz=42;   //常量表达式
    int arr[10];    //含有10个整数的数组
    int *parr[sz];  //含有42个元素的数组
    std::string bad[cnt]={"asd","dada"};   //错误，cnt不是常量表达式,能编译通过？？？？？？？？？？
    for (auto& x: arr){
        std::cout << x << std::endl;
    }

    int ial[sz]={0,1,2};    //含有3个元素，分别为0，1，2
    int a2[]={0,1,2};   //维度是3的数组
    int a3[5]={0,1,2};  //等价于 a3[]={0,1,2,3,4}
    std::string a4[3]={"hi","bye"}; //等价于a4[]={"hi","bye",""}
    // int a5[2]={0,1,2};  //错误：初始值过多


// //字符数组的特殊性
//     char a1[]={'c','+','+'};
//     char a2[]={'C','+','+','\0'};
//     char a3[]="C++";
//     // const char a4[6]="Daniel";  //错误：没有空间存放空字符

// //不允许拷贝和赋值
//     int a[]={0,1,2};
//     // int a2[]=a; //错误，不能使用一个数组初始化另一个数组
//     // a2=a;   //错误,不能把一个数组直接赋值给另一个数组

    int *ptrs[10];
    // int &refs[10]=/*?*/     //不存在引用的数组
    int (*Parray)[10]=&arr; 
    int (&arrRef)[10]=arr;

    
}