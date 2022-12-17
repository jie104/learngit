//
// Created by zxj on 2022/12/14.
//
#include <iostream>
#include <string>
#include <vector>

class Sales_data
{
public:
    Sales_data(std::string s,unsigned cnt,double price):
        bookNo(s),units_sold(cnt),revenue(cnt*price){
        std::cout << "bookNo: " << bookNo << " units_sold:" << units_sold
            << "revenue: " << revenue;
    }
    //委托构造函数
    Sales_data(): Sales_data("",0,0){}
    //抑制构造函数定义的隐式类型转换
    /*explicit*/ Sales_data(std::string s):Sales_data(s,0,0){}
    /*explicit*/ Sales_data(std::istream &is):Sales_data(){ read(is,*this);}

    std::string isbn() const {return bookNo;}
    Sales_data& combine(const Sales_data&);
    inline double avg_price() const;   //常量成员函数

    friend Sales_data add(const Sales_data&,const Sales_data&);
    friend std::ostream &print(std::ostream&,const Sales_data&);
    friend std::istream  &read(std::istream&,Sales_data&);


private:
    std::string bookNo;
    unsigned   units_sold;
    double revenue;
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

    //explict只能在类内声明构造函数时使用explict关键字
//    /*explicit*/ Sales_data::Sales_data(std::istream& is) { read(is,*this);}
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

class NoDefault
{
public:
//    NoDefault()=default;
    NoDefault(const std::string&){};
    NoDefault(int ){};

};

//struct A{
//    NoDefault my_mem;
//};
//
//A a;
//struct B{
//    B(): b_member("das"){}
//    NoDefault b_member;
//};

struct C{
    NoDefault c_member;
    C(): c_member(2){

    }
};

//聚合类
struct Data{
    int ival;
    std::string s;
};

//constexpr构造函数
class Debug{
public:
    constexpr Debug(bool b=true):hw(b),io(b),other(b){}
    constexpr Debug(bool h,bool i,bool o):
        hw(h),io(i),other(o){ }
    constexpr bool any(){return hw || io || other;}
    void set_io(bool b){io=b;}
    void set_hw(bool b){hw=b;}
    void set_other(bool b) {hw=b;}
private:
    bool hw;
    bool io;
    bool other;
};

//声明静态成员
class Account{
public:
    void calculate() {amount+=amount*interestRate;}
    static double rate() {return interestRate;}
    static void rate(double );
    static int apple;



private:
    std::string owner;
    double amount;
    static double interestRate;
    static double initRate();

    //可以为静态成员提供const整数类型的类内初始值，
    // 不过要求静态成员必须是字面值常量类型的constexpr
    static constexpr int period=30; //period是常量表达式
    double dialy_tbl[period];

};
//constexpr int Account::period=30;
//定义并初始化静态成员
//从类名开始，这条语句剩余部分位于类的作用域内，因此可以直接使用initRate()
double Account::interestRate=initRate();
int Account::apple=initRate();
void Account::rate(double newRate) {
    interestRate=newRate;
}

class Bar{
public:
    static Bar &getInstance(){
        try{
            static Bar bar;
            return bar;
        }catch(const std::exception& e){
            std::cout << "setting construct failed! " << e.what();
            throw e;
        }

    }
    int apple;

private:
    Bar(){}
    ~Bar(){}
    static Bar meml;    //正确：静态成员可以是不完全类型
    Bar *mem2;      //正确：指针成员可以是不完全类型
//    Bar mem3;       //错误：数据成员必须是完全类型
};

class Screen{
public:
    //可以使用静态成员作为默认实参
    Screen& clear(char=bkground);
private:
    static const char bkground;
};

int main()
{
    //声明一个返回Sales_data的函数obj()
    Sales_data obj();

    //NoDefault没有默认构造函数，将报错
//    std::vector<NoDefault> vec(10);
    std::vector<C> vec(10);

    std::string null_book="9-999-99999-9";
    Sales_data item;
    //隐式的类类型转换
    item.combine(null_book);
    item.combine(std::string("9-999-99999-9"));
    item.combine(Sales_data("9-999-99999-9"));
    item.combine(std::cin);

    Sales_data item1(null_book);    //正确，直接初始化
//    Sales_data item2=null_book; //错误：不能将explicit构造函数用于拷贝形式的初始化过程

    item.combine(Sales_data(null_book));

    //使用static_cast将std::cin强制转换为Sales_data
    item.combine(static_cast<Sales_data>(std::cin));

    Data val1={0,"Anna"};
    Account a;
    //直接用作用域运算符访问静态成员
    double r=Account::rate();

    Account ac1;
    Account *ac2=&ac1;
    //调用静态成员函数rate的等价形式
    r=ac1.rate();   //通过Account的对象或引用
    r=ac2->rate();  //通过指向Account对象的指针

    auto b=Bar::getInstance().apple;

}
