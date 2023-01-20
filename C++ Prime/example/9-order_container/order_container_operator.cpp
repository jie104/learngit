//
// Created by zxj on 2022/12/31.
//

#include<vector>
#include <iostream>
#include <list>
#include <forward_list>

int main()
{
#ifdef _READ_
    //元素访问
    std::vector<int> c={1,2,32,4,54,56,6};

    if(!c.empty()){
        auto val=*c.begin(),val2=c.front();
        auto last=c.end();
        auto val3=*(--last);    //不能递减forward_list迭代器
        auto val4=c.back();     //forward_list不支持
    }

    std::cout << c.back() << std::endl; //返回c中尾元素的引用。若c为空，函数行为未定义
    //返回c中首元素的引用。若c为空，函数行为未定义
    std::cout << c.front() << std::endl;
    //返回c中下标为n的元素的引用，n是一个无符号整数。若n>c.size()，则函数行为未定义
    std::cout << c[6] << std::endl;
    //返回下标为n的元素的引用。如果下标越界，则抛出以out_of_range异常
    std::cout << c.at(3) << std::endl;

    //在容器中访问元素的成员函数(即，front、back、下标、at)返回的都是引用
    //如果容器是一个const对象，则返回值是const的引用
    if(!c.empty()){
        c.front()=42;   //将42赋予c中的第一个元素
        auto &v=c.back();   //获得指向最后一个元素的引用
        v=1024; //改变c中的元素
        auto v2=c.back();
        v2=0;
    }

    //如果希望确保下标合法，可以使用at成员函数。at成员函数类似下标运算符，如果下标越界，at会抛出
    //一个out_of_range异常
    std::vector<std::string> svec;  //空vector
    std::cout << svec[0] << std::endl;   //运行时错误:svec中没有元素
    std::cout << svec.at(3) << std::endl;    //会抛出一个out_of_range异常
    std::cout << c.back() << std::endl;
    std::cout << c.front() << std::endl;

    //从容器内部删除一个元素，erase
    //删除list中的奇数
    std::list<int> lst={0,1,2,3,4,5,6,7,8,9};
    auto it=lst.begin();
    while (it!=lst.end()){
        if (*it%2){
            //erase返回指向删除的元素之后位置的迭代器
            it=lst.erase(it);
        }else{
            ++it;
        }
    }

    //forwrd_list
    std::forward_list<int> flst={0,1,2,3,4,5,6,7,8,9};
    auto prev=flst.before_begin();  //表示flst的“首前元素”
    auto curr=flst.begin();     //表示flst的第一个元素
    while (curr!=flst.end()){
        if (*curr%2){
            curr=flst.erase_after(prev);
        }else{
            prev=curr;
            ++curr;
        }
    }
#endif

    //改变容器大小
    //resize用来增大或缩小容器；如果当前大小大于所要求1的大小，容器后部的元素会倍删除；
    //如果当前大小小于新大小，会将元素添加到容器后部
    std::list<int> ilist(10,42);
//    ilist.clear();
    ilist.resize(10,0);
    std::cout << "begin to print the element of ilist" << std::endl;
    for (const auto &x:ilist){
        std::cout << x << std::endl;
    }

    //编写改变容器的循环程序
    //傻瓜循环，删除偶数元素，复制奇数元素
    std::vector<int> vi={0,1,2,3,4,5,6,7,8,9};
    auto iter=vi.begin();
    while (iter!=vi.end()){
        if (*iter%2){
            //insert(p,t)在迭代器p指向的元素之前创建一个值为t或由args创建的元素。
            //返回指向新添加的元素的迭代器
            iter=vi.insert(iter,*iter);
            iter+=2;
        }else{
            //erase删除迭代器p指定的元素，返回一个指向被删元素之后元素的迭代器
            iter=vi.erase(iter);
        }
    }

    //此循环的行为是未定义
    std::vector<int> v={0};
    auto begin=v.begin(),end=v.end();
    while (begin!=end){
        ++begin;
        begin=v.insert(begin,42);
        ++begin;
        std::cout << *begin << std::endl;
    }


}