//
// Created by zxj on 2023/1/5.
//

#include <iostream>
#include <vector>
#include<algorithm>
#include <string>
#include <list>

int main()
{
    int val=42;
    std::vector<int> vec={1,2,3,4,5,42};
    //如果在vec中找到想要的元素，则返回结果指向它，否则返回结果为vec.cend()
    //find将范围中的每个元素与定值进行比较，它返回指向一个等与给定值的元素的迭代器。
    //如果范围中无匹配元素，则find返回第二个元素来表示搜索失败
    auto result=std::find(vec.cbegin(),vec.cend(),val);
    std::cout << "The value " << val
              << (result==vec.cend()
                ?" is not present": " is present") << std::endl;

    //由于find操作的是迭代器，因此可以用同样的find函数在任何容器中查找值
    std::string val1="a value";
    std::list<std::string> lst={"sdaaf","daafafa","dafa"};
    auto result1=std::find(lst.cbegin(),lst.cend(),val1);

    //由于指针类似内置数组上的迭代器，可以用find来查找值
    int ia[]={27,210,21,2,1,2,3,21,83};
    int val2=83;
    int *result2=std::find(std::begin(ia),std::end(ia),val);

    //可在序列的子范围i查找
    auto result3=std::find(ia+1,ia+4,val);



}

