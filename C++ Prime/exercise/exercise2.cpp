#include <iostream>

int main(){
    int i,&ri=i;    //错误：引用类型的初始值必须是一个对象
    i=5;
    ri=10;
    int a=ri;
    std::cout << i << " " << ri << " " << a << std::endl;
}