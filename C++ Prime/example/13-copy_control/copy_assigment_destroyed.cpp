//
// Created by zxj on 2023/3/8.
//

#include <iostream>
#include <string>
#include <vector>

///拷贝构造函数
class Foo{
public:
    Foo();  //默认构造函数
    //拷贝构造函数第一个必须是一个引用类型，且参数几乎是const引用，
    // 拷贝构造函数了能会被隐式使用，因此不应该是explicit
    Foo(const Foo&);    //拷贝构造函数
};

//以Sales_data为例子，合成拷贝构造函数等价于
class Sales_data{
public:
    Sales_data(const Sales_data&);
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


void f(std::vector<int>);

///析构函数
//析构函数释放对象使用的资源，并销毁对象的非static数据对象
class Foo1 {
public:

    ~Foo1(); //析构函数
};


///使用=default
//通过将拷贝控制成员定义为=default来显式要求编译器合成的版本
class Sales_data{
public:
    //当在类内用=default修饰成员时，合成的函数将隐式地声明为内联
    Sales_data()=default;
    Sales_data(const Sales_data&)=default;
    Sales_data& operator=(const Sales_data&);
    ~Sales_data()=default;
};

//不希望合成的成员是内联函数，应该只对成员的类外定义使用=default，像对拷贝赋值运算符那样
Sales_data& Sales_data::operator=(const Sales_data &)=default;


///定义删除的函数
//新标准下，可将拷贝构造函数和拷贝赋值运算符定义为删除函数来阻止拷贝
//删除的函数，虽然声明了他们，但不能以任何方式使用它们
//在函数参数列表后面加上=delete来指出我们希望将它定义为删除
struct NoCopy{
    NoCopy()=default;   //使用合成默认构造函数
    NoCopy(const NoCopy&)=delete;   //阻止拷贝
    NoCopy &operator=(const NoCopy&)=delete;    //阻止赋值
    ~NoCopy()=default;  //使用合成的析构函数
};


///三五法则
/***
 * 如果一个类需要一个析构函数，几乎可以肯定它需要一个拷贝构造函数和一个拷贝赋值运算符
 */

int main()
{

///拷贝初始化
/***
 * 直接初始化，要求编译器使用普通的函数匹配来选择与我们
 * 提供的参数最匹配的构造函数；使用拷贝初始化时，要求
 * 编译器将右侧对象拷贝到正在创建的对象，如果有需要
 * 还要进行类型转换
 */
    std::string dots(10,'.');   //直接初始化
    std::string s(dots);    //直接初始化
    std::string s2=dots;    //拷贝初始化
    std::string null_book="9-999-99999-9";  //拷贝初始化
    std::string nines=std::string(100,'9'); //拷贝初始化


///拷贝初始化的限制
    //使用初始化要求通过一个explicit的构造函数来进行类型转换，
    //则使用拷贝初始化还是直接初始化就不是无关紧要
    std::vector<int> v1(10);    //正确，直接初始化
//    std::vector<int> v2=10; //错误，接受大小参数的构造函数是explicit
//    f(10);  //错误：不能用一个explicit的构造函数初始化
    f(std::vector<int>(10));    //从一个int直接构造一个临时vector

///编译器可以绕过拷贝构造函数
    //拷贝初始化过程中，编译器可以跳过拷贝/移动构造函数，直接创建对象
    //拷贝/移动构造函数必须是存在且可访问，不能是private
    std::string null_book0="9-999-99999-9";  //拷贝初始化
    std::string null_book1("9-999-99999-9");    //编译器略过拷贝构造函数



}

