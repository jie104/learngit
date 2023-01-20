//
// Created by zxj on 2022/12/20.
//

#include <iostream>
#include <fstream>

int main()
{
    //IO对象无赋值或i拷贝
//    std::ofstream out1,out2;
//    out1=out2;
//    //进行IO对象通常以引用方式传递和返回流
//    //读写一个IO对象会改变其状态，因此传递和操作的引用不能是const
//    std::ofstream print(std::ofstream);
//    out2=print(out2);

    //IO操作一个与生俱来的问题是可能发生错误，一些错误是可以恢复，而其他错误
    //则发生在系统深处，已经超出应用程序可以修正的范围
    int ival;
    std::cin >> ival;

    char word;
    while (std::cin >> word){

    }

    auto old_state=std::cin.rdstate();  //记住cin当前状态
    std::cin.clear();   //将流中所有状态复位，将流状态设置为有效
    std::cin.setstate(old_state);   //将cin置为原有状态


    //IO库定义了4个iostate类型的constexpr值,表示特定的位模式
    //这些值用来表示特定类型的IO条件，可以与位运算符一起使用，来一次
    //检测或设置多个标志位
    std::cin.failbit;
    std::cin.badbit;
    std::cin.eofbit;
    std::cin.goodbit;

    //复位failbit和badbit，保持其他标志位不变
    std::cin.clear(std::cin.rdstate() & ~std::cin.failbit & ~std::cin.badbit);

    std::cout << "hi!" << std::endl;    //输出hi和一个换行，然后刷新缓冲区
    std::cout << "hi!" << std::flush;   //输出hi,然后刷新缓冲区，不加任何额外字符
    std::cout << "hi!" << std::ends;    //输出hi和一个空字符，然后刷新缓冲区

    std::cout << std::unitbuf;  //所有输出操作后都会立即刷新缓冲区
    std::cout << std::nounitbuf;    //回到正常的缓冲方式

    std::cout.tie();
    std::cin.tie();

    //cin不再与其他流关联
    std::ostream *old_tie=std::cin.tie(nullptr);

    //将cin与cerr关联；这不是一个好主意，因为cin应该关联到cout
    std::cin.tie(&std::cerr);
    std::cin.tie(old_tie);  //重建cin与cout间的正常关系

}
