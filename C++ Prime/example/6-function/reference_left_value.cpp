//
// Created by zxj on 2022/11/23.
//

#include <iostream>
#include<string>


char &get_val(std::string &str,std::string::size_type ix){
    return str[ix];
}

std::vector<std::string> process(){
    std::string expected="dssf";
    std::string actual="efwd";
    if (expected.empty())
        return {};
    else if (expected==actual)
        return {"functionX","okay"};
    else
        return {"functionX",expected,actual};
}

int main()
{
    std::string s("a value");
    decltype(s.size()) a=3;
    std::cout << s << std::endl;
    get_val(s,0)='A';
    std::cout << s << std::endl;
    return 0;
}
