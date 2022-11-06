//exercise 3.3.3

#include <iostream>
#include<string>
#include <vector>

int main()
{
    std::string s;
    std::vector<std::string>  v;
    while (std::cin >> s){
        v.push_back(s);
        
        if (s=="q"){
            break;
        }
    }

    for (int i=0;i<v.size();i++){
        for (auto it=v[i].begin();it!=v[i].end();it++){
            *it=toupper(*it);
        }

    }

    std::cout << "begin to print the element of v";
    for (int k=0;k<v.size();k++){
        std::cout << v[k] << std::endl;
    }


    std::vector<int> e(10,42);  //创建指定数量的元素
    std::vector<int> e1={42,42,42,42,42,42,42,42,42,42};    //列表初始化
    std::vector<int> e2=e;

}