//
// Created by zxj on 2023/2/15.
//

//#define _MAKE_SHARED_
//#define _DIRECT_MANAGE_MEMORY_
#define _WEAK_PTR_

#include <iostream>
#include <memory>
#include <string>
#include <list>
#include <vector>
#include <algorithm>
#include <initializer_list>
#include "head_file/StrBlob.h"
#include <new>
#include "head_file/StrBlobPtr.hpp"

///传递unique_ptr参数和返回unique_ptr
/***
 * 不能拷贝unique_ptr的规则有个例外，可以拷贝或赋值一个将要被销毁的unique_ptr
 *
 */
std::unique_ptr<int> clone(int p){
    //正确：从一个int*创建一个unique_ptr<int>
    return std::unique_ptr<int>(new int(p));
}

std::unique_ptr<int> clone1(int p){
    std::unique_ptr<int> ret(new int(p));
    return ret;
}



///智能指针和异常
//智能指针与哑类
struct destination; //表示正在链接什么
struct connection;  //使用连接所需的信息
connection connect(destination*);   //打开连接
void disconnection(connection);     //关闭给定连接

void f(){
    std::shared_ptr<int> sp(new int(42));
}

//void f1(destination &d){
//    connection c= connect(&d);
//    std::unique_ptr<connection, decltype(end_connection)*>
//            p(&c,end_connection);
//}


struct example{
    int apple;
    double banal;
    example(int a,double b):apple(a),banal(b){}
};

template <class T>
struct Blob{
    T s;
};


//正确
std::shared_ptr<int> clone3(int ptr){
    return std::shared_ptr<int>(new int(ptr));
}

void process(std::shared_ptr<int> ptr){
    std::cout << "...." << *ptr << std::endl;
}

int main()
{
//    std::shared_ptr<std::string> p1;    //shared_ptr,可以指向string
//    std::shared_ptr<std::list<int>> p2; //shared_ptr,可以指向int的list
//    if(p1 && p1->empty()){
//        *p1="hi";
//    }
//    std::shared_ptr<std::string> p3;
//    *p3="daad";
//
//    std::list<int> l={1,2,3};
//    *p2=l;
//
//    std::cout <<p1.get() <<std::endl;
//    //交换指向同种类型的p1、p3中的指针
//    p1.swap(p3);

#ifdef _MAKE_SHARED_
///make_shared函数
/***
 类似顺序容器emplace成员，make_shared用其参数来构造给定类型的对象
 */
    //指向一个值为42的int的shared_ptr
    std::shared_ptr<int> p4=std::make_shared<int>(42);
    //p5指向一个值为"9999999999"的string
    std::shared_ptr<std::string> p5=std::make_shared<std::string>(10,'9');
    //p6指向一个值初始化的int,即，值为0
    std::shared_ptr<int> p6=std::make_shared<int>();
    auto p7=std::make_shared<std::vector<std::string>>();

    std::cout << "*p4: " << *p4 << std::endl;
    std::cout << "*p5: " << *p5 << std::endl;
    std::cout << "*p6: " << *p6 << std::endl;

    auto p8=std::make_shared<std::vector<std::string>>();
#endif

    //shared_ptr的拷贝和构造
    auto p=std::make_shared<int>(42);
    auto q(p);

    std::vector<int> v={1,4,3,2,7};
    std::nth_element(v.begin(),v.begin()+v.size()/2,v.end());
    std::cout << v[v.size()/2] << std::endl;

    ///使用动态生存期的资源的类
    std::vector<std::string> v1;
    {
        std::vector<std::string> v2={"a","an","the"};
        v1=v2;
    }//v2被销毁，其中的元素也被销毁
    //v1有三个元素，是原来v2中元素的拷贝

    //某些类分配的资源具有原对象独立的生存期
//    Blob<std::string > b1;
//    {
//        Blob<std::string> b2={"a","an","the"};
//    }

    StrBlob b1;
    {
        StrBlob b2={"a","an","the"};
        b1=b2;
        b2.push_back("about");
        std::cout << "b2 size: " << b2.size() << std::endl;
    }
    //共享指针
    std::cout << "b1 size: " << b1.size() << std::endl;  //4

#ifdef _DIRECT_MANAGE_MEMORY_
///直接管理内存
    /***
     *  默认情况下，动态分配的对象是默认初始化，这意味着内置类型或组合类型的对象的值将是未
        定义的，而类类型对象将用默认构造函数进行初始化
     */
     std::string *ps=new std::string;   //初始化为空string
     int *pi=new int;   //pi指向一个未初始化的int

     //使用直接初始化来初始化一个动态分配的对象
     int *pi1=new int(1024); //pi指向的对象的值为1024
     std::string *ps1=new std::string(10,'9');

     std::vector<int> *pv=new std::vector<int>{0,1,2,3,4,5,6,7,8,9};

     std::string *ps2=new std::string;  //默认初始化为空string
     std::string *ps3=new std::string();    //值初始化为空string
     int *pi3=new int;  //默认初始化，*pi3的值未定义
     int *pi4=new int();    //值初始化为0；*pi4为0

     auto *StrBlob_ptr=new StrBlob({"ada","daa"});
     auto *example_ptr=new example(2,1);
     std::cout << example_ptr->apple << "," << example_ptr->banal << std::endl;

     //auto初始化器来推断我们想要分配的对象的类型
     auto p1=new auto(1);
    //由于编译器要用初始化器的类型来推断分配的类型，只有当括号中仅有单一初始化器时才能使用auto
   // auto p2=new auto(1,1.2);

///动态分配的const对象
    /***
     * 类似其他任何const对象，一个动态分配的const对象必须初始化。对于一个定义的
     * 默认构造函数的类类型，其const动态对象可以隐式初始化，而其他类型的对象必须
     * 显式初始化
     */
    //分配并初始化一个const int
    const int *pci=new const int(1024);
    //分配并初始化一个const的空string
    const std::string *pcs=new const std::string;
#endif

#ifdef _DELETE_
///内存耗尽
    int *p1=new int;    //如果分配失败，new抛出std::bad_alloc
    int *p2=new (std::nothrow)int;  //如果分配失败，new返回一个空指针

///释放动态内存
    delete p1;   //p必须指向一个动态分配的对象或一个空指针

///指针值和delete
/***
 * 编译器不能分辨一个指针指向的是以静态还是动分配的对象。
 * 类似，编译器不能分辨一个指针所指向的内存是否已经释放
 * 对于这些delete表达式，大多数编译器会编译通过，尽管是错误
 */
    int i,*pi1=&i,*pi2= nullptr;
    double *pd=new double (33),*pd2=pd;
    delete i;   //错误，i不是指针
    delete pi1;    //未定义：pi1指向一个局部变量
    delete pd;  //正确
    delete pd2; //未定义：pd2指向的内存已经释放
    delete pi2; //正确：释放一个空指针总是没错

    delete pci; //正确，释放一个const对象

    //提供有限的保护
    int *p5(new int(42));
    auto q5=p5;  //p，q指向相同内存
    delete p5;  //p5,q5均变无效
    p5= nullptr;    //指出p5不再绑定到任何对象

#endif

#ifdef _TECHNIQUE_PTR_AND_NEW_
///shared_ptr与new结合
    //可以用new返回的指针来初始化智能指针
    std::shared_ptr<double> p1;
    std::shared_ptr<int> p2(new int(42));   //p2指向一个值为42的int

    /***
     * 接受指针参数的智能指针构造函数是explicit。因此不能将
     * 一个内置指针隐式转换为一个智能指针，必须直接使用初始化形式
     */
    //错误：必须使用直接初始化
//     std::shared_ptr<int> p3=new int(1024);
     std::shared_ptr<int> p4(new int(1024));    //正确：使用直接初始化

     /***
      * 同样的，一个返回shared_ptr的函数不能在返回语句中隐式转换一个普通指针
      */
    //错误：隐式转换为shared_ptr<int>
//      std::shared_ptr<int> clone(int p){
//          return new int(p);
//
//      }
    std::cout <<"*p4:  " << *p4 <<std::endl;
    p4.reset(new int(12));
    std::cout <<"*p4:  " << *p4 <<std::endl;

///不要混合使用普通指针和智能指针
    std::shared_ptr<int> p5(new int(42));   //引用计数为1
    process(p); //拷贝p会递增它的引用计数，在process中引用计数为2
    int i=*p;   //正确：引用计数值为1

    //传递给process一个临时shared_ptr，可能会导致错误
    int *x(new int(1024));
    std::cout << "*x: " << *x << std::endl;

    process(std::shared_ptr<int>(x));   //合法，但内存会被释放
    int j=*x;   //未定义：x是一个空悬指针
    std::cout << "j: " << j << std::endl;


///不要使用get初始化另一个智能指针或为智能指针赋值
    /***
     * get用来将指针的访问权限传递给代码，只有在确定代码不会delete指针的
     * 情况下，才能使用get。特别是，永远不要使用get初始化另一个智能指针
     * 或者为另一个智能指针赋值
     */
    std::shared_ptr<int> p6(new int(42));    //引用计数为1
    int *q1=p.get();
    {
        //程序块结束，q被销毁，它指向的内存被释放
        std::shared_ptr<int>(q1);
    }
    int foo=*p; //未定义:p指向的内存已经释放

///其他shared_ptr操作
    p.reset(new int(1024)); //正确：p指向一个新对象

    if(!p.unique()){
        p.reset(new int(*p));
    }
    *p+=2;
    std::shared_ptr<int> p6(new int(42));
    process(std::shared_ptr<int>(p6));
    process(std::shared_ptr<int>(p6.get()));

    auto p7=new int(12);
    auto sp=std::make_shared<int>();
    process(sp);
//    process(new int()); //动态指针不能隐式转化为智能指针
//    process(p7);    //动态指针不能隐式转化为智能指针
    process(std::shared_ptr<int>(p7));  //执行函数，p7会被释放，有安全隐患

    std::cout << "p7: " << *p7  <<std::endl;

    auto sp1=std::make_shared<int>(12);
    std::cout << "*sp1: " << *sp1 << std::endl;
    auto sp2=sp1.get();
    delete sp2;
    //此时sp1内存被释放掉了
    std::cout << "*sp1: " << *sp1 << std::endl;
#endif

#ifdef _UNIQUE_PTR_
///unique_ptr
    /***
     * 与shared_ptr不同，某个时刻只能有一个unique_ptr指向一个指定对象。
     * 当unique_ptr被销毁时间，它指向的对象也被销毁
     *定义一个unique_ptr时，需要将其绑定到一个new返回的指针
     */
    std::unique_ptr<double> p1(new double(4)); //可以指向一个double的unique_ptr
    std::cout << "p1: " << *p1 <<std::endl;
    std::unique_ptr<int> p2(new int(42));   //p2指向一个值为42的int
    std::cout << "p2: " << *p2 <<std::endl;

    //由于一个unique_ptr拥有它所指向的对象，因此unique_ptr不支持普通的拷贝和赋值
    std::unique_ptr<std::string> p3(new std::string("Stegosaurus"));
//    std::unique_ptr<std::string> p4(p3);    //错误：unique_ptr不支持拷贝
    std::unique_ptr<std::string> p5;
//    p5=p4;      //错误：unique_ptr不支持赋值

    //可以通过调用release或reset将指针的所有权从一个非const unique_ptr转移到unique
    //将所有权从p1转移到p2
    std::unique_ptr<std::string> p6(p3.release());
    std::cout << "p6: " << *p6 <<std::endl;
    std::cout << "p3: " << *p3 <<std::endl;
    std::unique_ptr<std::string> p7(new std::string("Trex"));
    //将所有权从p7转移到p6
    p6.reset(p7.release()); //reset释放了p2原来指向的内存

    //调用release会切断unique_ptr和它管理的对象间的联系。
    //release返回的指针通常被用来初始化另一个智能指针或給另一个智能指针赋值
    //如果我们不用另一个智能指针来保存release返回的指针，我们的程序就要负责释放资源
    p2.release();   //错误：p2不会释放内存，且丢失了指针
    auto p8=p2.release();    //正确，但必须记得delete(p)

#endif

#ifdef _WEAK_PTR_
    //当创建一个weak_ptr时，需要用shared_ptr来初始化
    auto p9=std::make_shared<int>(42);
    std::weak_ptr<int> wp(p9);   //wp弱共享p；p的引用计数未改变

    if (std::shared_ptr<int> np=wp.lock()){ //若np不空则条件成立

    }

#endif

}