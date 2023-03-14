//
// Created by zxj on 2023/3/9.
//
#include <iostream>
#include <string>

class Sales_data{
public:
    Sales_data()=default;
    Sales_data(const Sales_data&);
    Sales_data& operator=(const Sales_data&);
private:
    std::string bookNo;
    int units_sold=0;
    double revenue=0.0;
};

Sales_data::Sales_data(const Sales_data &orig):
        bookNo(orig.bookNo),    //使用string的拷贝构造函数
        units_sold(orig.units_sold),    //拷贝orig.units_sold
        revenue(orig.revenue)       //拷贝orig.revenue
{    }

///等价于合成拷贝赋值运算符
Sales_data&
Sales_data::operator=(const Sales_data &rhs){
    bookNo=rhs.bookNo;  //调用string::operator
    units_sold=rhs.units_sold;  //使用内置的int赋值
    revenue=rhs.revenue;    //使用内置的double赋值
    return *this;   //返回此对象的引用
}

///重载赋值运算符
/***
 * 赋值运算符必须定义为成员函数，其左侧运算对象就绑定到隐式的this参数
 *  赋值运算符通常返回指向左侧运算对象的引用
 */

class Foo{
public:
    Foo& operator=(const Foo&); //赋值运算符
};

int main()
{
    Sales_data trans,accum;
    trans=accum;    //使用Sales_data的拷贝赋值运算符


}