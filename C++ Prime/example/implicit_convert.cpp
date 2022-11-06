#include <iostream>


int main()
{
//大多数用到数组的表达式中，数组自动转化成指向首元素的指针
//当数组被用作decltype关键字的参数，或作为取地址符(&)，sizeof及typeid等运算符的
//运算对象时，上述转换不会发生
    int b[4]={1,2,3,4};
    decltype(b) c;
    auto d=&b;  
    auto e=sizeof(b);
    std::cout << b << std::endl;
    std::cout << **d << std::endl;
    // auto e=d;


//转换成布尔类型
//如果指针或算术类型的值为0，转换结果为0，否则是true
    int num=1;
    char *cp=0;
    if (!cp){
        std::cout << "cp_ptr is null" << std::endl;
    }
    int *cp1=&num;
    if (!cp1){
        std::cout << "cp1_ptr is null" << std::endl;
    }

//转换成常量
//允许将指向非常量类型的指针指向相应向量类型的指针，对于引用也是如此
    int i;
    const int &j=i; //非常量类型转换成const int的引用
    const int *p=&i;    //非常量类型地址转换成const的地址

}