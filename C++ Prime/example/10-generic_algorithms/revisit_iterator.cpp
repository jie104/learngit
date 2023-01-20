//
// Created by zxj on 2023/1/28.
//

#include <iterator>
#include <vector>
#include <iostream>
#include <list>
#include <algorithm>

int main()
{
//插入迭代器
    std::vector<int> c={1,2,3,4,5,6,7};
    int val=4;
    /***
        inserter创建一个使用insert的迭代器，此函数接受第二个参数，这个参数必须是一个指向给定
        容器的迭代器。元素将被插入到给定迭代器所表示的元素之前
     ***/
     auto iter=c.begin()+2;
     auto it=std::inserter(c,iter);
     *it=val;
     *it=34;
     for (auto &x:c){
         std::cout << "c: " << x << std::endl;
     }

     std::list<int> lst={1,2,3,4};
     std::list<int> lst2,lst3;
     std::copy(lst.cbegin(),lst.cend(),std::front_inserter(lst2));
     std::copy(lst.cbegin(),lst.cend(),std::inserter(lst3,lst3.begin()));

     for(auto &x:lst2){
         std::cout << "lst2: " << x << std::endl;
     }

     for(auto &x:lst3){
         std::cout << "lst3: " << x << std::endl;
     }

}

