//
// Created by zxj on 2022/11/27.
//

#include <iostream>
#include <string>

typedef std::string::size_type sz;
//将函数指定为内联函数，通常就是将他在每个调用节点上“内联地”展开
//一般来说，内联机制用于优化规模较小，流程直接，频繁调用的函数
//内联说明只是说明向编译器发出的一个请求，编译器可以选择忽略这个请求
inline const std::string& screen(const std::string& s,sz ht=24,sz wid=80,char backrnd=' ');
sz wd=80;
char def=' ';
sz ht();
//声明默认实参初始值
std::string screen(sz =ht(),sz =wd,char =def);

int main()
{
    const std::string& window=screen("sda");
    std::cout << window << std::endl;

    std::string window1= screen();


}

inline const std::string &screen(const std::string& s,sz ht,sz wid,char backrnd){
    return s;
}

void f2(){
    def='*';
    sz wd=100;  //隐藏外层定义的wd，但没有定义默认值
    std::string window1=screen();   //调用screen(ht(),80,'*')
}

