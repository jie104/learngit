#include <iostream>
#include <string>

int main()
{
//任何具有明确定义的类型转换，只要不包含低层const，都可以使用static_cast
    int i,j;
    double slope=i/j;

//static_cast
//任何具有明确定义的类型转换，只要不包含低层const，都可以使用static_cast
    double slope=static_cast<double>(j)/i;

//当需要把一个较大的算术类型赋值给较小的类型时，static_cast非常有用
//当执行了显式的类型转化后，警告信息会被关闭

//static_cast对于无法自动执行类型转换也非常有用
//可以使用static_cast找回存在于void*指针
    double d;
    void *p=&d; //正确：任何非常量对象的地址都能存入void*
    //正确：将void*转换回初始的指针类型
    double *dp=static_cast<double*> (p);

//const_cast
//const_cast只能改变运算对象的低层const
    const char *pc;
    char *p=const_cast<char*>(pc);  //正确：但通过p写值是未定义的行为

//只有const_cast能改变表达式的常量属性，使用其他形式的命名强制类型转换
//改变表达式的常量属性都将引发编译器错误
    const char *cp2;
    const_cast<char*>(cp2); 

    //错误：static_cast不能转换掉const性质
    // char *q=static_cast<char*>(cp2);
    static_cast<std::string>(cp2);
    



}
