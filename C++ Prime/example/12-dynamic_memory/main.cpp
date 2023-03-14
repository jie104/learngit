//
// Created by zxj on 2023/2/27.
//
#include "head_file/QueryResult.h"
#include "head_file/TextQuery.h"

void runQueries(std::ifstream &infile){
    //infile是ifstream，指向我们要处理的文件
    TextQuery tq(infile);   //保存文件并查询map
    //与用户交互：提示用户输入要查询的单词，完成查询并打印结果
    while (true){
        std::cout << "enter word to look for,or q to quit: ";
        std::string s;
        //若遇到文件尾或用户输入‘q’时，循环终止
        if (!(std::cin >>s) || s=="q") break;
        //指向查询并打印结果
        print(std::cout,tq.query(s)) << std::endl;
    }
}


int main()
{
    std::ifstream fcin("/home/zxj/data/other/example.txt");
    runQueries(fcin);
}

