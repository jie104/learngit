//
// Created by zxj on 2023/2/16.
//

#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <new>

std::vector<int> * f(){
    std::vector<int> *fv_ptr=new std::vector<int>();
    return fv_ptr;
}

std::vector<int> * g(){
    auto gv_ptr=f();
    gv_ptr->resize(10);
    for(auto beg=gv_ptr->begin();beg!=gv_ptr->end();beg++){
        std::cin >> *beg;
    }
//    int size=gv_ptr->size();
//    for(int i=0;i<size;i++){
//        std::cin >> *gv_ptr[i];
//    }
    return gv_ptr;

}

void h(){
    auto hv_ptr=g();
    for (auto &x:*hv_ptr){
        std::cout << "打印的数字： " << x << std::endl;
    }
    delete hv_ptr;
}

std::shared_ptr<std::vector<int>> sf(){
    std::shared_ptr<std::vector<int>> sfv_ptr(new std::vector<int>());
    return sfv_ptr;
}
std::vector<int> * sg(){
    auto sgv_ptr=f();
    sgv_ptr->resize(5);
    for(auto beg=sgv_ptr->begin();beg!=sgv_ptr->end();beg++){
        std::cin >> *beg;
    }
//    int size=gv_ptr->size();
//    for(int i=0;i<size;i++){
//        std::cin >> *gv_ptr[i];
//    }
    return sgv_ptr;

}

void sh(){
    auto hv_ptr=sg();
    for (auto &x:*hv_ptr){
        std::cout << "打印的数字： " << x << std::endl;
    }
}

int *b(){
    int *p=new int;//此时p指向的int未定义
    return p;
}


int main()
{
    //exercise 12.6
//    h();

    //exercise 12.7
//    sh();

    std::cout << *b() << std::endl;

    //exercise 12.9
    int *q=new int(42),*r=new int(100);
    r=q;
    std::cout << "*r: " << *r << std::endl;
    auto q2=std::make_shared<int>(42),r2=std::make_shared<int>(100);
    r2=q2;
    std::cout << "*r2: " << *r2 << std::endl;

    //exercise 12.17
    int ix=1024,*pi=&ix,*pi2=new int(2048);
    typedef std::unique_ptr<int> IntP;
//    IntP p0(ix);
//    IntP p1(pi);
//    std::cout << "*p1: " << *p1 << std::endl;
    IntP p2(pi2);
//    IntP p3(&ix);
    IntP p4(new int(2048));
//    IntP p5(p2.get());

}
