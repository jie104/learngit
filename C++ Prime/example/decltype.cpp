//2.5.3、decltype类型指示符
//有时我们希望，从表达式的类型推断出要定义的变量的类型，但不想用该表达式的值初始化变量

#include <iostream>

int f( ){
    return 1;
}
int main(){
    decltype(f()) sum=1;    //sum的表达式是函数f的返回类型

    const int ci=0,&cj=ci;
    decltype(ci) x=0;   //x的类型是const int
    decltype(cj) y=x;   //y的类型是 const int&,y绑定到变量x
    // decltype(cj) z;     //错误：z是一个引用，必须初始化


//如果decltype使用的表达式不是一个变量，则decltype返回结果对应的类型
//decltype的结果可以是引用类型
    int i=42,*p=&i,&r=i;
    decltype(r+0) b;    //正确：加法的结果是int，因此b是一个(未初始化)int
    // decltype(*p) c; //如果表达式的内容是解引用操作，则decltype将得到引用类型，因此c是int&,必须初始化


//如果给变量加上了一层或多层括号，编译器会把他当作一个表达式
//变量是一种可以作为赋值语句左值的特殊表达式，所以decltype会得到一个引用类型
    //decltype的表达式如果是加上了括号的变量，结果将是引用
    decltype((i)) d;    //错误：d是int&,必须初始化
    decltype(i) e;  //正确：e是一个(未初始化)int
}
