//
// Created by zxj on 2023/3/9.
//

#include<iostream>
#include <string>
#include <memory>
#include <vector>

//exercise 13.5
class HasPtr{
public:
    HasPtr(const std::string &s=std::string()):
        ps(new std::string(s)),i(10){}
    HasPtr &operator=(const HasPtr& hasPtr){
        ps=hasPtr.ps;
        i=hasPtr.i;
        return *this;
    }


    ~HasPtr(){
        delete ps;
    }
    HasPtr(const HasPtr &hasPtr):
        ps(new std::string(*hasPtr.ps)),i(hasPtr.i)
    {

    }
private:
    std::string *ps;
    int i;
};

//exercise 13.13
struct X{
    X():a(2),b(3),c(4)
        {std::cout << "X()" << std::endl;}
    X(const X&){std::cout << "X(const X&) " << std::endl;}
    X &operator=(X &Y){
        a=Y.a;
        b=Y.b;
        c=Y.c;
        return *this;
    }
    ~X(){}
    int a,b,c;
};
typedef std::shared_ptr<X> X_ptr;

void useX(X x){
    X_ptr z(new X);
    std::vector<X_ptr > vector_X;
    vector_X.emplace_back(z);
    for (auto &x:vector_X){
        std::cout << x->a << " " << x->b << " " << x->c << std::endl;
    }
}

//此代码导致hp上的string指针被delete两次
HasPtr f(HasPtr hp){    //HasPtr是传值参数，所以将被拷贝
    HasPtr ret=hp;      //拷贝给定的HasPtr
    return ret;     //ret和hp被清空
}

//exercise 13.14
class numbered{
public:
    int mysn;
    numbered(){

    }


    numbered(const numbered &n){
        std::cout << "mysn: " << mysn << std::endl;
    }

};

void f( numbered s){
    std::cout << s.mysn << std::endl;
}

//exercise 13.18
class Emplpyee{
public:
    Emplpyee(){
        id_++;
        std::cout << "id: " << id_ << std::endl;
    }

    Emplpyee(std::string name):name_(name){
        id_++;
        std::cout << "id: " << id_ << std::endl;

    }

    Emplpyee(const Emplpyee& emplpyee):name_(emplpyee.name_)
    {

    }

    Emplpyee &operator=(const Emplpyee& emplpyee){
        name_=emplpyee.name_;
        return *this;
    }
    ~Emplpyee(){}



private:
    std::string name_;
    static unsigned id_;//TODO：静态成员类外定义


};
    unsigned Emplpyee::id_=0;


int main(){
//    X x;
//    X y;
//    y.a=10;
//    y.b=20;
//    y.c=30;
//    x=y;
//    std::cout << "a: " << x.a << " b: " << x.b << " c: " << x.c << std::endl;
//    useX(y);
//
//    HasPtr p("some values");
//    f(p);
//    HasPtr q(p);
    //TODO
    numbered a,b=a,c=b;
    f(a);
    int l=1;
    f(b);
    f(c);

    Emplpyee one,two;
    Emplpyee three("jie"),four("jocon");


}