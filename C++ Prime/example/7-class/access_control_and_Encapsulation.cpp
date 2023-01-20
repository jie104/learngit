//
// Created by zxj on 2022/12/7.
//
#include <string>
#include <iostream>

class Sales_data
{
    friend Sales_data add(const Sales_data&,const Sales_data&);
    friend std::ostream &print(std::ostream&,const Sales_data&);
    friend std::istream  &read(std::istream&,Sales_data&);

public:
    Sales_data()=default;
    Sales_data(const std::string& s,unsigned n,double p):
            bookNo(s),units_sold(n),revenue(p*n){ }
    Sales_data(const std::string& s):bookNo(s) { }
    Sales_data(std::istream&);
    std::string isbn() const {return bookNo;}
    Sales_data& combine(const Sales_data&);
    Sales_data& combine(Sales_data&);
    Sales_data &combine(const Sales_data&) const;

private:
    double avg_price() const{
        return units_sold? revenue/units_sold:0;
    }
    std::string bookNo;
    unsigned units_sold=0;
    double revenue=0.0;
};

Sales_data add(const Sales_data&,const Sales_data&);
std::ostream &print(std::ostream&,const Sales_data&);
std::istream  &read(std::istream&,Sales_data&);


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

