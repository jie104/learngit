//
// Created by zxj on 2022/11/19.
//

#include <iostream>
#include <string>
#include <ctype.h>
#include <vector>

bool is_empty(const std::string& s);
void lowAph(std::string& s);
std::string lowAph(const std::string& s);
std::vector<int> change_val(int n,std::vector<int>& v);


bool isUpAph(const std::string s);
int main()
{
    //exercise 6.16
    is_empty("wdad");

    //exercise 6.17

    std::cout << isUpAph("sAadf") << std::endl;

    std::cout << lowAph("AdadasfS") << std::endl;

}

bool is_empty(const std::string& s){
    return s.empty();
}

bool isUpAph(const std::string s){
    for (const auto& x:s){
        if(isupper(x)){
            return true;
        }

    }
    return false;
}

std::string lowAph(const std::string& s){
    std::string s1=s;
    for (auto& x:s1){
        x=tolower(x);
    }
    return s1;

}

std::vector<int> change_val(int n,std::vector<int>& v){

}

