//
// Created by zxj on 2022/11/30.
//

#include <iostream>
#include <string>

class Sales_data{
public:
    //构造函数
    //当某个数据成员被构造函数初始值列表忽略时，它将以与合成默认构造函数相同的方式初始化，如下
    Sales_data( std::string& s):bookNo(s) {}
    //定义默认构造函数，令其与只接受一个string实参数的构造函数功能相同
//    Sales_data(std::string s=""):bookNo(s){}
    //Sales_data(const std::string& s):
      //      bookNo(0),units_sold(0),revenue(0){}

    Sales_data(const std::string& s,unsigned n,double p):
            bookNo(s),units_sold(n),revenue(p*n){}
    Sales_data(std::istream& is=std::cin);

    friend Sales_data add(const Sales_data&,const Sales_data&);
    friend std::ostream &print(std::ostream&,const Sales_data&);
    friend std::istream  &read(std::istream&,Sales_data&);
    //定义在类内部的函数是隐藏的inline函数
    std::string isbn() const {return bookNo;}
    Sales_data& combine(const Sales_data&);
    inline double avg_price() const;   //常量成员函数

    //委托构造函数
    Sales_data(): Sales_data("",0,0){}
    Sales_data(std::string s):Sales_data(s,0,0){}
    Sales_data(std::istream &is) : Sales_data(){
        read(is,*this);
    }


private:
    std::string bookNo;
    unsigned units_sold=0;
    double revenue=0.0;
};

Sales_data add(const Sales_data&,const Sales_data&);
std::ostream &print(std::ostream&,const Sales_data&);
std::istream  &read(std::istream&,Sales_data&);

inline double Sales_data::avg_price() const {
    if (units_sold)
        return revenue/units_sold;
    else
        return 0;
}

Sales_data& Sales_data::combine(const Sales_data & rhs) {
    units_sold+=rhs.units_sold; //把rhs成员加到this对象成员
    revenue+=rhs.revenue;
    return *this;   //返回调用该函数的对象
}

std::istream &read(std::istream& is,Sales_data& item){
    double price=0;
    is >> item.bookNo >> item.units_sold >> price;
    item.revenue=price*item.units_sold;
    return is;
}

std::ostream &print(std::ostream& os,const Sales_data &item){
    os << item.isbn() << " " << item.units_sold << " "
       << item.revenue << " " << item.avg_price();
    return os;
}

Sales_data add(const Sales_data& lhs,const Sales_data &rhs){
    Sales_data sum =lhs;
    sum.combine(rhs);
    return sum;
}

Sales_data::Sales_data(std::istream& is){
    read(is,*this); //read函数的作用是从is中读取一条交易信息
                            //然后存入this对象
}

Sales_data first_item(std::cin);

int main()
{
//    Sales_data next;
    Sales_data last("9-999-99999-9");
}