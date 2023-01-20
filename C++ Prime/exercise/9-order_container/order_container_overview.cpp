//
// Created by zxj on 2022/12/27.
//

#include <iostream>
#include<vector>    //可变大小数组，支持快速随机访问。尾部之外的维护i插入或删除元素可能很慢
#include<deque> //双端队列。支持快速随机访问。再头尾位置插入/删除速度很快
#include<list>  //双向链表
#include <forward_list> //单向链表
#include <array>    //固定大小数组
#include <string> //与vector相似容器，但专门用于保存字符

class Sales_data{

};

bool is_find(const std::vector<int>& v,int k){
    auto begin=v.begin();
    auto end=v.end();
    for (;begin!=end;begin++){
        if (*begin==k){
            return true;
        }
    }
    return false;
}

int findInt(const std::vector<int>& v,int k){
    auto begin=v.begin();
    auto end=v.end();
    for (;begin!=end;begin++){
        if (*begin==k){
            std::cout << "find the value:" << *begin << std::endl;
            return *begin;
        }
    }
    std::cout << "k is not exist,will return 0";
    return 0;
}

bool is_vector_equal(const std::vector<int> &v1,const std::vector<int> &v2){
    if (v1==v2){
        std::cout << "v1==v2" << std::endl;
        return true;
    }
    std::cout << "v1!=v2" << std::endl;
    return false;
}
int main()
{
#ifdef _NINE_ONE_
    std::list<Sales_data> l;    //保存Sales-data对象的list
    std::deque<double> d;   //保存double的deque

    std::vector<std::vector<std::string> > lines;    //vector的vector
//    std::vector<Sales_data> v1(10,init);
    std::vector<Sales_data> v2(10);
    v2.resize(10);

    std::list<std::deque<int> > deques;
    std::vector<int> v={12,3,4,54,65};
    v.resize(32,1);
    v={1,2,2,34};
    findInt(v,4);

    std::list<int> lst1;
    std::list<int>::iterator iter1=lst1.begin(),
                             iter2=lst1.end();

    //迭代器的常规运算只能应用于string,vector,dequ和array的迭代器
//    while(iter1 <iter2){
//
//    }

    //容器定义和初始化
    //将一个容器初始化为另一个容器得拷贝
    std::list<std::string> authors={"Milton","Shakespares","Austen"};
    std::vector<const char*> articles={"a","an","the"};

    //创建一个容器为另一个容器得拷贝，两个容器的类型及元素类型必须一致
    std::list<std::string> list2(authors);  //正确：类型匹配
    std::list<std::string> list3=authors;  //正确：类型匹配
//    std::deque<std::string> authList(authors);  //容器类型不匹配
//    std::vector<std::string> words(articles);   //容器类型不匹配

    //当传迭代器参数来拷贝一个范围时，不要求容器类型一致；
    //新容器和原容器中的元素类型可以不同，只要能将要拷贝得元素转换为要初始化的元素的容器的元素类型
    //可以将const char*元素转化为string
    std::forward_list<std::string> words1(articles.begin(),articles.end());

    auto it=--authors.end();
    std::deque<std::string> aythList(authors.begin(),it);

//与顺序容器大小相关的构造函数
    //顺序容器(array除外）还提供另一个构造函数，它接受一个容器大小和一个元素初始值
    //如果不提供元素初始值，则标准库会创建一个值初始化器
    std::vector<int> ivec(10,-1);
    ivec={4,5,6,7,8};
    std::list<std::string> svec(10,"hi!");
    std::forward_list<int> ivec1(10);
    ivec1={123,4543};
    std::deque<std::string> svec2(10);

    //标准库array具有固定大小
    //与内置数组一样，标准库array的大小也是类型的一部分
    std::array<int,10> ia1; //10个默认初始化的int
    std::array<int,10> ia2={0,1,2,3,4,5,6};
    std::array<int,10> ia3={42};

    //虽然不能对内置数组类型进行拷贝或对象赋值操作，但array并无此限制
    int digs[10]={0,1,2,3,4,5,6,7,8,9};
//    int cpy[10]=digs;   //错误，内置数组不支持拷贝或赋值
    std::array<int,10> digits={0,1,2,3,4,5,6,7,8,9};
    std::array<int,10> copy=digits;

    std::array<int,10> a1={0,1,2,2,3,3,4,54};
    std::array<int,10> a2={0};  //所有元素均为0
    a1=a2;
    //由于右边运算对象大小可能与左边运算对象大小不同，因此array类型不支持assign，也不允许用
    //花括号包包围的值列表进行赋值
//    a2={0,1};

    std::list<std::string> names;
    std::vector<const char*> oldstyle;
//    names=oldstyle;//错误：容器类型不匹配
    //assign允许我们从一个不同但相容的类型赋值，或从容器的一个子序列赋值
    names.assign(oldstyle.cbegin(),oldstyle.cend());
    std::list<std::string> slist1(1);
    slist1.assign(10,"Hiya!");

    //swap，除array外，交换两个容器的内容的操作保证会很快，元素本身并未交换
    //swap只是交换了两个容器得内部数据结构
    std::vector<std::string> svec1(10);
    std::vector<std::string> svec4(24);
    std::cout << "svec1: " << svec1.size() << std::endl;
    std::cout << "svec4: " << svec4.size() << std::endl;
    std::swap(svec1,svec4);
    std::cout << "svec1: " << svec1.size() << std::endl;
    std::cout << "svec4: " << svec4.size() << std::endl;

    //exercise 9.2.5
    std::list<char*> l1;
    std::vector<std::string> v1;
    v1.assign(l1.begin(),l1.end());

    //exercise 9.11
    std::vector<int> v_obj={1,2,3,3,4,5,6,67,7};

    std::vector<int> v_initialize;  //默认初始化
    std::vector<int> v_initialize1={1,2,23,4};  //列表初始化
    //与顺序容器大小相关的构造函数
    std::vector<int> v_initialize2(10,1);
    std::vector<int> v_initialize3(10);
    std::vector<int> v_initialize4=v_obj;   //拷贝初始化，两个容器类型及其元素类型必须匹配
    //当传递迭代器参数来拷贝一个范围时，不要求容器类型是相同，且新容器和原容器中的元素类型可以不同
    //只要能将要拷贝的元素转换
    std::vector<int> v_initialize5(v_obj.begin(),v_obj.end());

    //exercise 9.13
    std::list<int> l_={1,2,3,3,4,5,6};
    std::vector<int> v_(1,3);
    std::vector<double> v_1(l_.begin(),l_.end());
    std::vector<double> v_2(v_.begin(),v_.end());

    //exercise 9.14
    std::list<char*> l_c;
    std::vector<std::string> v_s;
    v_s.assign(l_c.begin(),l_c.end());

    //exercise 9.15
    std::vector<int> v_i;
    std::vector<int> v_i1;
    is_vector_equal(v_i,v_i1);

    //exercise 9.16
    std::vector<int> v_16;
    std::list<int> l_16;
    std::vector<int> v_temp(l_16.begin(),l_16.end());
    is_vector_equal(v_16,v_temp);

#endif

#ifdef _NINE_THIRDTY_ONE_
    //exercise 9.31
    std::list<int> vi={0,1,2,3,4,5,6,7,8,9};
    auto iter=vi.begin();
    while (iter!=vi.end()){
        if (*iter%2){
            //insert在给定位值之前插入新元素，然后返回指向新插入元素的迭代器
            iter=vi.insert(iter,*iter);
            iter++;
            iter++;
        } else{
            //删除当前迭代器指向的元素，并返回该迭代器的后面的迭代器
            iter=vi.erase(iter);
        }
    }
    for (auto &x:vi){
        std::cout << "vi: " << x << std::endl;
    }


    std::forward_list<int> flst={0,1,2,3,4,5,6,7,8,9};
    auto prev=flst.before_begin();
    auto iter1_begin=flst.begin();
    while (iter1_begin!=flst.end()){
        if (*iter1_begin%2){
            //insert在给定位值之前插入新元素，然后返回指向新插入元素的迭代器
            iter1_begin=flst.insert_after(iter1_begin,*iter1_begin);
            prev=iter1_begin;
            iter1_begin++;
        } else{
            //删除当前迭代器指向的元素，并返回该迭代器的后面的迭代器

            iter1_begin=flst.erase_after(prev);
        }
    }
    for (auto &x:flst){
        std::cout << "flst: " << x << std::endl;
    }
#endif

#ifdef _NINE_THIRDTY_THREE_
    //exercise 9.33
    std::list<int> v={0,1,2,3,4,5,6,7,8,9};
    auto begin=v.begin();
    while (begin!=v.end()){
        ++begin;
        begin=v.insert(begin,42);
        ++begin;
    }
    for (auto &x:v){
        std::cout << "v: " << x << std::endl;
    }
#endif

#ifdef _NINE_THIRDTY_FOUR_
    //exercise 9.34
    std::list<int> vi={0,1,2,3,4,5,6,7,8,9};
    auto iter=vi.begin();
    while (iter!=vi.end()){
        if (*iter%2){
            iter=vi.insert(iter,*iter);
            ++iter;
            ++iter;
        }else{
            ++iter;
        }
//        std::cout << 111111111111111 << std::endl;
    }
    for (auto &x:vi){
        std::cout << "vi: " << x << std::endl;
    }

#endif

    //exercise 9.39
    std::vector<std::string> svec;
    svec.reserve(1024);
    std::string word;
    while (std::cin >> word && word!="q"){
        svec.push_back(word);
    }
    for (auto &x:svec){
        std::cout << "svec: " << x << std::endl;
    }

    svec.resize(svec.size()+svec.size()/2);
    for (auto &x:svec){
        std::cout << "svec: " << x << std::endl;
    }


}
