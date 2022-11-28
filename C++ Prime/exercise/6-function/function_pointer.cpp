//
// Created by zxj on 2022/11/29.
//

#include <iostream>
#include <vector>

//int f(int,int);
int add(int,int);
int sub(int,int);
int mul(int,int);
int divi(int,int);
typedef int (*pf)(int,int);


int main()
{
    std::vector<pf> v;
    auto pf1=&add;
    v.emplace_back(pf1);
    pf1=&sub;
    v.emplace_back(pf1);
    pf1=&mul;
    v.emplace_back(pf1);
    pf1=&divi;
    v.emplace_back(pf1);

    for (const auto& f:v){
        std::cout << f(1,2) << std::endl;
    }




}
int add(int x,int y){
    return x+y;
}

int sub(int x,int y){
    return x-y;
}

int mul(int x,int y){
    return x*y;
}

int divi(int x,int y){
    if (y!=0){
        return x/y;
    }else{
        return 0;
    }
}



