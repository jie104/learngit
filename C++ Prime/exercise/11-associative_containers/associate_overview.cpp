//
// Created by zxj on 2023/2/4.
//

#include <vector>
#include <utility>
#include <string>
#include <iostream>
#define _WAY_TWO_ 0

int main()
{
    //exercise 11.12
    std::string s;
    int n;
    typedef std::pair<std::string,int> string_int;
    std::vector<string_int > v;

    while(std::cin >> s && s!="q"){
        std::cin >> n;

#if _WAY_ONE_
        std::cout << "way one " << std::endl;
        string_int p(s,n);
//        p.first=s;
//        p.second=n;
#elif _WAY_TWO_
        std::cout << "way two " << std::endl;
        string_int p=std::make_pair(s,n);
#else
        std::cout <<"way three " << std::endl;
        string_int p{s,n};
#endif
        v.push_back(p);


    }
    for (auto &x: v){
        std::cout << "first: " << x.first << ",second: " << x.second << std::endl;
    }
}
