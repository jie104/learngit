//
// Created by zxj on 2022/12/18.
//

#include <iostream>
#include <fstream>
#include <string>

std::istream &read(std::istream & s){
    char s1;
    while (s >> s1 && !s.eof()){
        if(!s){
            s.clear();
        }
        std::cout << "s1: " << s1 << std::endl;
    }
    std::cin.clear(std::cin.rdstate() & ~std::cin.failbit & ~std::cin.badbit);
    return s;
}

int main()
{
    read(std::cin);
}