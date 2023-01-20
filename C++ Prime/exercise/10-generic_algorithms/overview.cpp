//
// Created by zxj on 2023/1/6.
//

#include <algorithm>
#include <vector>
#include <string>
#include <iostream>
#include <list>

int main()
{
    //exercise 10.1
    int val=3;
    std::vector<int> v={1,2,3,4,5,6,67,8,3,2,3};
    //count返回定值在迭代器中出现的次数
    std::cout << std::count(v.begin(),v.end(),val) << std::endl;

    //exercise 10.2
    std::string s="dasda";
    std::list<std::string> lst={"dasda","dasfaf","gfretger"};
    std::cout << std::count(lst.begin(),lst.end(),s) << std::endl;


}