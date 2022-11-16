//
// Created by zxj on 2022/11/13.
//

#include <iostream>

//通过调用运算符来执行函数。调用运算符的形式是一对圆括号，他作用于一个表达式
//该表达式是函数或指向函数的指针：圆括号内是一个用逗号隔开的实参列表
int fact(int);
double FABS(double );
int main()
{
    std::cout <<  fact(3) << std::endl;
    int j=fact(5);
    std::cout << "5! is " << j << std::endl;
    int val;
    std::cin >> val ;
    std::cout << fact(val) << std::endl;
    std::cout << FABS(-30.2) << std::endl;
    return 0;
}

int fact(int val){
    int ret=1;
    while (val>1){
        ret*=val--;
    }
    return ret;

}

double FABS(double b){
    if (b >0){
        return b;
    }else{
        return -b;
    }
}
