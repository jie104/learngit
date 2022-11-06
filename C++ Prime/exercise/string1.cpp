
#include <iostream>
#include <string>

int main()
{
//exercise3.6
#ifdef _STRING_1_
    std::string s="adsadsafsaga";
    for (auto& x:s){
        x='X';
    }
    std::cout << s << std::endl;


//exercise3.7
    std::string s="frevwvwvb";
    for (decltype(s.size()) i=0;i<s.size();i++){
        s[i]='X';
    }
    std::cout << s << std::endl;
//exercise 3.8
    std::string s="fewfgwgs";
    decltype(s.size()) i=0;
    while (i < s.size()){
        s[i]='X';
        i++;
    }
    std::cout << s << std::endl;
//exercise 3.10
    std::string s="Hello,I'm xiaoming";
    std::string s1;
    for (auto& c:s){
        if (ispunct(c)){
            continue;
        }
        s1+=c;
        
    }
    std::cout << s1 << std::endl;
#endif
//exercise3.11
    const std::string s="Keep out!";
    for (auto &c:s){

    }
}