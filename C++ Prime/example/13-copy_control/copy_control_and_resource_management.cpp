//
// Created by zxj on 2023/3/20.
//
#include <string>
#include <vector>
#include <new>
//#include <glog/logging.h>
#include <iostream>

///行为像值的类
/***
 * 为提供类值的行为，对于类管理的资源，每个对象都应该有自己的拷贝
 */
class HasPtr{
public:
    //构造函数分配新的string和新的计数器，将计数器置为1
    HasPtr(const std::string &s=std::string()):
            ps(new std::string(s)),i(0),use(new std::size_t(1)){}


    //拷贝构造函数拷贝所有三个成员，并递增计数器
    HasPtr(const HasPtr &hasPtr):
            ps(new std::string(*hasPtr.ps)),i(hasPtr.i),use(hasPtr.use)
    {
        ++*use;
    }

    //先递增rhs中的计数，然后再递减左侧运算对象中的计数来实现这一点
//    HasPtr &operator=(const HasPtr& hasPtr);

//在赋值运算符中使用swap
    HasPtr &operator=(HasPtr rhs);

    ~HasPtr();

    ///除了定义拷贝控制成员，管理资源的类通常定义一个名为swap的函数
    friend void swap(HasPtr&,HasPtr&);
private:
    std::string *ps;
    int i;
    std::size_t *use;   //用来记录有多少个对象共享*ps的成员
};

inline void swap(HasPtr &lhs,HasPtr &rhs){
    using std::swap;
    swap(lhs.ps,rhs.ps);
    swap(lhs.i,rhs.i);
    swap(lhs.use,rhs.use);
}

//这个技术有趣之处是它处理自赋值情况且天然就是异常安全，
// 它通过在改变左侧运算对象之前拷贝右侧运算对象保证了自赋值的正确
HasPtr& HasPtr::operator=(HasPtr rhs) {
    //交换左侧运算对象和局部变量的rhs的内容
    swap(*this,rhs);    //rhs现在指向本对象曾经使用的内存
    return *this;   //rhs被销毁，从而delete了rhs中的指针
}

//此处考虑两个对象相同的情况
//HasPtr& HasPtr::operator=(const HasPtr &rhs) {
//    ++*rhs.use; //递增右侧运算对象的引用计数
//    if(--*use ==0){ //递减本对象的引用计数
//        delete ps;  //如果没有其他用户，释放本对象的成员
//        delete use; //释放本对象分配1的成员
//    }
//    ps=rhs.ps;  //将数据从rhs拷贝到本对象
//    i=rhs.i;
//    use=rhs.use;
//    return *this;   //返回本对象
//
//}

HasPtr::~HasPtr() {
    if (--*use==0){ //如果引用计数变为0
        delete ps;  //释放string内存
        delete use; //释放计数器内存
    }
}

    ///类值拷贝赋值运算符
    /***
     * 当编写一个赋值运算符时，一个好的模式是先将右侧运算对象拷贝到一个局部临时对象中
     * 当拷贝完成后，销毁左侧运算对象的现有成员就安全。一旦左侧运算对象资源被销毁，
     * 就只剩下将数据从临时对象拷贝到左侧运算对象的成员中
     * @param rhs
     * @return
     */
//    HasPtr &HasPtr::operator=(const HasPtr &rhs) {
//        auto newp=new std::string(*rhs.ps); //拷贝底层string
//        delete ps;  //TODO:必须释放该资源，否则当另一个指针指向他时，该内存将被一直占用
//        ps=newp;
//        i=rhs.i;
//        std::cout << "begin to copy";
//        return *this;   //返回本对象
//
//    }

class Foo{
public:
    friend void swap(Foo &lhs,Foo &rhs);

private:
    HasPtr h;
};

//每个swap调用应该都是未加限定的，即每个调用应该都是swap，而不是std::swap
//如果存在类型特定的swap版本，其匹配程度会优于std中定义的版本
void swap(Foo &lhs,Foo &rhs){
    using std::swap;
    swap(lhs.h,rhs.h);  //调用HasPtr版本的swap
//    std::swap(lhs.h,rhs.h);   //调用标准库版本的swap
}



int main(){
    HasPtr a("apple");
    HasPtr b;
    b=a;
}



