#include <iostream>
#include <vector>
#include <string>


std::string sa[10];
int ia[10]; //全局变量初始值默认为0
int main()
{
#ifdef _INBUILT_ARRAY_
//exercise 3.5.1
    unsigned buf_size=1024;
    int ia[buf_size];
    int ia[4*7-14];
    // int ia[txt_size()];
#endif

//exercise 3.28
    std::string sa2[10];
    int ia2[10];    //·局部变量初始值随机赋予

    for (auto& x: ia2){
        std::cout << x << std::endl;
    }


//1、不能直接用数组赋值
//2、数组元素个数固定
//3、数组维数必须是常量表达式
}