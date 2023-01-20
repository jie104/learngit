//
// Created by zxj on 2023/2/6.
//

#include <map>
#include <vector>

int main(){
    //exercise 11.15
    std::map<int,std::vector<int>>::mapped_type imaped_type;
    std::map<int,std::vector<int>>::key_type ikey_type;
    std::map<int,std::vector<int>>::value_type ivalue_type;

    //exercise 11.16
    std::map<int,int> imap;
    auto iter=imap.begin();
    iter->second=3;
    //iter->first=3;  //对map而言，得到的是一个pair类型的引用，其first成员保存const关键字


}

