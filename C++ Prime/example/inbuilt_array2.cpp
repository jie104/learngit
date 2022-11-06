#include <iostream>
#include <cstring>
#include <vector>

int main()
{
    char ca[]={'c','+','+'};    //不以空字符结束
    std::cout << std::strlen(ca) << std::endl;  //严重错误，没有以空字符结束

//比较字符串
    const char ca1[]="A string example";
    const char ca2[]="A different string";
    if (ca1 < ca2){ //试图比较两个无关的地址，将导致未定义

    }
    //想比较两个C风格字符串需要用strcmp函数
    if (strcmp(ca1,ca2)<0){

    }

    std::string s("Hello World");
    // char *str=s;    //错误：不能用string对象初始化char*
    //c_str函数的返回值是一个C风格字符串
    const char *str=s.c_str();  //正确
    
//允许使用数组来初始化vector对象，只需指明要拷贝的首元素地址和尾后元素地址
    int int_arr[]={0,1,2,3,4,5};
    std::vector<int> ivec(std::begin(int_arr),std::end(int_arr));
    for (int i=0;i<6;i++){
        std::cout << ivec[i] << std::endl;
    }
    std::vector<int> subVec(int_arr+1,int_arr+4);
    for (int i=0;i<3;i++){
        std::cout << subVec[i] << std::endl;
    }
}