//
// Created by zxj on 2022/12/27.
//

#include <vector>
#include <iostream>
#include <list>
#include<string>

class Sales_data{
public:
    Sales_data()=default;
    Sales_data(std::string s1,int t1,float f1):s(s1),t(t1),f(f1){}
    Sales_data(std::string s1):s(s1){}

private:
    std::string s;
    int t;
    float f;
};

int main()
{
    std::vector<int> v={1,2,3,4,5};
    auto begin=v.begin();
    auto end=v.end();

    while (begin!=end){
        *begin=4;
        ++begin;
    }

    std::vector<int>::value_type x;

    std::list<std::string> a={"Milton","Shakespeare","Austen"};
    auto it1=a.begin();
    auto it2=a.rbegin();
    auto it3=a.cbegin();
    auto it4=a.crbegin();

    //显式指定类型
    std::list<std::string>::iterator it5=a.begin();
    std::list<std::string>::const_iterator it6=a.begin();

    //是iterator还是const_iterator依赖于a的类型
    auto it7=a.begin();
    auto it8=a.cbegin();

    std::vector<int> v1;
    const std::vector<int> v2;
    auto it11=v1.begin();
    auto it21=v2.begin();
    auto it31=v1.cbegin(),it41=v2.cbegin();

    std::vector<int> v3(3);
    std::vector<int> v8(3,7);
    //v3初始化为v4的拷贝,v3,v4是相同类型，即他们必须是相同的容器类型，且保存
    std::vector<int> v4(v3);//v4=v3;

    std::vector<int> v5={1,2,3,4,5};
    std::vector<int> v6{1,2,3,4,5};
    auto beg=v5.begin();
    auto end1=v5.end();
    //c初始化为迭代器b和e指定范围中元素的拷贝
    std::vector<int> v7(beg,end1);
    for(auto &x:v7){
        std::cout << x << std::endl;
    }

    //除forward_list外，每个容器类型支持三种大小操作size(),empty(),max_size()
    //forward_list支持max_size().empty(),但不支持size
    v7.size();
    v7.empty();
    v7.max_size();

    //关系运算符
    //除了无序关联容器外，所有容器都支持关系运算符
    std::vector<int> h1={1,3,5,7,9,12};
    std::vector<int> h2={1,3,9};
    std::vector<int> h3={1,3,5,7};
    std::vector<int> h4={1,3,5,7,9,12};
    h1 <h2;
    h1 <h3;
    h1==h4;
    h1==h2;

    //只有当其元素类型也定义了相应得比较运算符时，我们才可以使用关系运算符来比较两个容器
    std::vector<Sales_data> storeA,storeB;
//    if (storeA <storeB){
//
//    }

    //push_back
    //当我们使用一个对象来初始化容器时，或将一个对象插入到容器时，实际上放入到容器中得是一个拷贝
    //而不是对象本身
    //除array和forward_list外，每个顺序容器都支持push_back
//    std::string word;
//    std::list<std::string> container;
//    while (std::cin >> word){
//        container.push_back(word);
//    }

    //push_front
    //list,forward_list和deque容器支持push_front
    std::list<int> ilist;
    //将元素添加到ilist开头
    for (std::size_t ix=0;ix!=4;++ix)
        ilist.push_back(ix);

    //vector、deque、list和string支持insert成员，forward_list支持了特殊版本的insert成员
    //insert函数将元素插入到迭代器指定的位置之前
    std::list<std::string> slist={"a","b"};
    auto iter=slist.end();
    slist.insert(iter,"Hello!");
    for(const auto &x:slist){
        std::cout << x << std::endl;
    }

    std::vector<std::string> svec;
    slist.insert(slist.begin(),"Hello!");

    //vector不支持push_front,但可以插入到begin()之前
    //vector插入到末尾之外的任何位置都可能很慢
    svec.insert(svec.begin(),"Hello!");
    svec.insert(svec.end(),10,"Anna");

    std::vector<std::string> h7={"quasi","simba","frollo","scar"};
    //将v的最后两个元素添加到slist的开始位置
    slist.insert(slist.begin(),h7.end()-2,h7.end());
    //新标准下。接受元素个数或范围的insert版本返回指向第一个新加入的元素的迭代器
    auto x6=slist.insert(slist.end(),{"these","wrods","will","go",
                                "at","the","end"});

    //运行时错误：迭代器要拷贝的范围，不能指向与目的位置系相同的容器
//    slist.insert(slist.begin(),slist.begin(),slist.end());

    //通过使用insert返回值，可以在容器中一个特定位置反复插入元素
    std::list<std::string> lst1;
    auto iter1=lst1.begin();
    std::string word;
    while(std::cin >> word){
        iter1=lst1.insert(iter1,word);  //相当于调用push_back
    }

    //使用emplace操作
    //当我们调用一个emplace成员函数时，则将参数传递给元素类型的构造函数
    //emplace成员使用这些参数在容器管理的内存空间中直接构造元素
    //当调用pus或insert成员函数时，将元素类型的对象传递给他们，这些对象被拷贝到容器中
    std::vector<Sales_data> c;
    c.emplace_back("978-041414411",25,15.99);
    //错误：没有接受三个参数的push_back版本
//    c.push_back("dassdafa",25,15.99);
    c.push_back(Sales_data("41414",23,12.2));

    //emplace函数的参数根据元素类型而变化，参数必须与元素类型的构造函数相匹配
    c.emplace_back();   //使用Sales_data的默认构造函数
    auto iter2=c.end();
    c.emplace(iter2,"999-9999999"); //作用类似insert



}