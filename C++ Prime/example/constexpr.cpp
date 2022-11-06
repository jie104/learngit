#include <iostream>

//constexpr常量表达式是指不会改变并且在编译过程就能得到计算结果的表达式
//字面值属于常量表达式，用常量表达式初始化的const对象也是常量表达式
int out=1;
constexpr int i=42;   //i的类型是整型常量
int j=0;

int main()
{
    const int max_files=20;     //max_files是常量表达式
    const int limit=max_files+1;    //limit是常量表达式
    int staff_size=27;  //staff_size不是常量表达式
    // const int sz=get_size();    //不是常量表达式

//声明为constexpr的变量是一定是一个常量，必须用常量表达式初始化
    constexpr int mf=20;    //20是常量表达式
    constexpr int limit=mf+1;   //mf+1是常量表达式
    constexpr int sz=size();    //只有当size是一个constexpr函数时，才是一条正确的声明语句

//一个constexpr的初始值必须是nullptr或0，或者存储于某个固定地址的对象
//固定地址即指针是常量指针
    constexpr int *a=nullptr;
    constexpr int *b=0;
//函数体内的变量一般并非存放在固定地址中，因此constexpr不能指向这样的变量
//相反，函数体外的对象其他地址固定不变
    constexpr int *c=&out;  //out是定义在函数体外的向量，地址不变
    constexpr int *d=&mf;   //函数体内的变量一般并非存放在固定地址

//在constexpr声明中如果定义了一个指针，限定符constexpr仅对指针有效，与指针所指对象无关
    const int *p=nullptr;   //p是一个指向整型常量的指针
    constexpr int *q=nullptr;   //q是一个指向整数的常量指针

    constexpr const int *p=&i;  //p是常量指针,指向整型常量i
    constexpr int *p1=&j;       //p1是常量指针，指向整数j

}

constexpr int size(){
    return 2;
}
