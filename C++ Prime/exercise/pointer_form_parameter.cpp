//
// Created by zxj on 2022/11/17.
//

#include <iostream>
void exchange(int& ,int&);

int main()
{
    int a=3,b=4;
    exchange(a,b);
    std::cout << "a: " << a << std::endl;
    std::cout << "b: " << b << std::endl;

}

void fcn(const int i){

}

void fcn(int i){

}
void exchange(int &a,int &b){
    int temp=a;
    a=b;
    b=temp;
}