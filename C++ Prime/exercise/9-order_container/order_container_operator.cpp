//
// Created by zxj on 2022/12/31.
//

#include <iostream>
#include <string>
#include <deque>
#include <list>
#include <vector>
#include <forward_list>
#include <algorithm>

template <class T>
void print_container_element(const T& container){
    for(const auto &x:container){
        std::cout << x << std::endl;
    }
}

void forward_list_find_insert(std::forward_list<std::string>& flst,const std::string &s1,const std::string &s2){
    auto it=std::find(flst.begin(),flst.end(),s1);
    if (it!=flst.end()){
        flst.insert_after(it,s2);
    }else{
//        bool flag= false;
        std::forward_list<std::string>::iterator it1=flst.before_begin();
        for (auto &x:flst){
//            if(flag){
//                ++it1;
//            }
//            flag= true;
            ++it1;
        }
        flst.insert_after(it1,s2);
    }
}

int main() {

#ifdef _NINE_ONE_EIGHT_
    //exercise 9.18
    std::string s;
    std::deque<std::string> Deque;

    while (std::cin >> s && s!="q" ){
        Deque.emplace_back(s);
    }
    for (const auto &x:Deque){
        std::cout << x << std::endl;
    }
#endif

#ifdef _NINE_ONE_NINE_

    //exercise 9.19
    std::string s1;
    std::list<std::string> List;

    while (std::cin >> s1 && s1!="q" ){
        List.emplace_back(s1);
    }
    for (const auto &x:List){
        std::cout << x << std::endl;
    }

#endif

#ifdef _NINE_TWENTY_


    //exercise 9.20
    std::deque<int> deque_even;
    std::deque<int> deque_odd;
    std::list<int> List_20={1,2,3,3,4,5,6,6,87,3};

    for (const auto &x:List_20){
        if (x%2==0){
            deque_even.emplace_back(x);
        }else{
            deque_odd.emplace_back(x);
        }
    }

    print_container_element(deque_odd);
    print_container_element(deque_even);

#endif

#ifdef _NINE_TWENTY_ONE_
    //exercise 9.21
    std::vector<std::string> lst;
    auto iter=lst.begin();
    std::string word;

    while (std::cin >> word){
        iter=lst.insert(iter,word);
    }
#endif

#ifdef _NINE_TWENTY_TWO_
    //exercise 9.22
    std::vector<int> iv;
    std::vector<int>::iterator iter=iv.begin(),
                                mid=iv.begin()+iv.size()/2;
    int some_val=2;

    while (iter!=mid){
        if (*iter ==some_val){
            iter=iv.insert(iter,2*some_val);
        }
    }

    //exercise 9.25
    std::list<int> lst={1,2,3,4,5,6,7,8,9,10};
    auto elem1=++lst.begin();
    auto elem2=++lst.begin();
    //当elem1与elem2非尾后迭代器，且相等时，返回当前元素的迭代器
    auto result=lst.erase(elem1,elem2);
    std::cout << "result: " << *result << std::endl;
    std::cout << "size: " << lst.size() << std::endl;

    //当elem3和elem4为尾后迭代器，且相等时，返回最后一个元素
    auto elem3=lst.end();
    auto elem4=lst.end();
    auto result1=lst.erase(elem3,elem4);
    std::cout << "result1: " << *result1 << std::endl;

#endif

#ifdef _NINE_TWENTY_SIX_
    //迭代器的强大之处
    //exercise 9.26
    int ia[] = {0, 1, 1, 2, 3, 5, 8, 13, 21, 55, 89};
    std::vector<int> v(std::begin(ia), std::end(ia));
    std::list<int> lst(std::begin(ia), std::end(ia));

    auto beg = v.begin();
    while (beg != v.end()) {
        if (*beg % 2) {
            beg = v.erase(beg);
        } else {
            ++beg;
        }
    }
    for (auto &x:v) {
        std::cout << "v: " << x << std::endl;
    }

    auto beg1 = lst.begin();
    while (beg1 != lst.end()) {
        if (*beg1 % 2 == 0) {
            beg1 = lst.erase(beg1);
        } else {
            ++beg1;
        }
    }
    for (auto &x:lst) {
        std::cout << "lst: " << x << std::endl;
    }

#endif

#ifdef _NINE_TWENTY_SEVEN_
    //exercise 9.27
    std::forward_list<int> flst = {0, 1, 2, 3, 3,  4, 5, 6, 67, 7, 8};
    auto prev = flst.before_begin();
    auto curr = flst.begin();
    while (curr != flst.end()) {
        if (*curr % 2){
            curr = flst.erase_after(prev);
        }else{
            prev = curr;
            ++curr;
        }
    }
    for (auto &x:flst){
        std::cout << "flst: " << x << std::endl;
    }
    std::forward_list<std::string> flst1={"dadafa","afafaf","dafaffa"};
    std::string s1="afafaf",s2="124141";

    forward_list_find_insert(flst1,s1,s2);
    for (auto &x:flst1) {
        std::cout << "flst1: " << x << std::endl;
    }
#endif

    //当用resize增大、缩小容器时，array不支持resize；
    //如果当前大小大于所要求的大小，容器后部的元素会被删除；
    //如果当前大小小于新大小，会将新元素添加到容器后部
    std::vector<int> vec;
    int i=0;
    while (vec.size()<25){
        vec.emplace_back(i*i);
        ++i;
    }
    for (auto &x:vec){
        std::cout << "vec: " << x << std::endl;
    }

    vec.resize(100,12);
    int j=0;
    for (auto &x:vec){
        std::cout << "vec1: " << j << ": " << x << std::endl;
        ++j;

    }

    vec.resize(10);
    int k=0;
    for (auto &x:vec){
        std::cout << "vec2: " << k << ": " << x << std::endl;
        ++k;

    }


}