//
// Created by zxj on 2023/2/14.
//

#include <iostream>
#include <unordered_map>
#include <string>
#include <unordered_set>

struct Sales_data
{
    int ISBN=2;
    int isbn(){
        return ISBN;
    }
};

//TODO
size_t hasher(const Sales_data &sd)  {
    return std::hash<std::string>()(sd.isbn());
}

bool eqOp(const Sales_data &lhs,const Sales_data &rhs){
    return lhs.isbn() ==rhs.isbn();
}




int main()
{
    std::unordered_map<std::string,size_t> word_count;
    std::string word;
    while (std::cin >> word && word!="q")
        ++word_count[word];
    for (const auto &x:word_count){
        std::cout << x.first <<" occurs " << x.second
                  << ((x.second)>1 ? " times" :" time") << std::endl;
    }

    //无序容器对关键字类型的要求
    //可以直接定义关键字是内置类型，string还是只能指针类型的无序容器
    std::unordered_set<Sales_data, decltype(hasher)*, decltype(eqOp)*>;
}

