//
// Created by zxj on 2022/11/17.
//
#include <iostream>
#include <string>

std::string::size_type find_char(const std::string &s,char c,
                                 std::string::size_type &occurs){
    auto ret =s.size();
    occurs=0;
    for (decltype(ret) i=0;i!=s.size();++i){
        if (s[i]==c){
            if (ret == s.size())
                ret=i;
            ++occurs;
        }
    }
    std::cout << ret << std::endl;
}

int main()
{
    std::string s="dfasfasfgaagegewo";
    std::string::size_type ctr=0;
    auto index= find_char(s,'o',ctr);
    std::cout <<  index << std::endl;
    std::cout << ctr << std::endl;
}