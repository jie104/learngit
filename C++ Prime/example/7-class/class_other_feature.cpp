//
// Created by zxj on 2022/12/8.
//

#include <iostream>
#include <string>
#include <vector>

class Screen{
public:
    //定义类型的成员，必须先定义后使用，与普通成员有所区别
    typedef std::string::size_type pos;
    Screen()=default;   //因为Screen有另一个构造函数，所以本函数是必需的
    Screen(pos ht,pos wd):height(ht),width(wd),contents(""){}
    //cursor被类内初始值初始化为0
    Screen(pos ht,pos wd,char c):height(ht),width(wd),contents(ht*wd,c){}
    //读取光标字符，隐式内联
    char get() const{return contents[cursor];}
    inline char get(pos ht,pos wd) const;   //显式内联
    Screen &move(pos r,pos c);  //能在之后被设为内联
    void some_memeber() const;

    //Window_mgr的成员可以访问Screen类的私有部分
    friend class Window_mgr;
//    friend void clear(SreenIndex);


    Screen& set(char);
    Screen& set(pos,pos,char);
    //根据对象是否是const重载display函数
    Screen& display(std::ostream& os){ do_display(os); return *this;}
    const Screen& display(std::ostream& os) const{
        display(os);
        return *this;
    }

    pos size() const;


private:
    void do_display(std::ostream& os) const {os << contents;}
    pos cursor=0;
    pos height=0,width=0;
    std::string contents;
    mutable size_t access_str;
};

Screen::pos Screen::size() const {
    return height*width;
}




void Screen::some_memeber() const {
    ++access_str;
}
//inline成员函数应该与相应类定义在同一个头文件
//可在函数定义处指定inline
inline Screen &Screen::move(pos r, pos c) {
    pos row=r*width;
    cursor=row+c;
    return *this;
}
//函数内部声明成inline
char Screen::get(pos r,pos c) const {
    pos row=r*width;
    return contents[row+c];
}

inline Screen &Screen::set(char c) {
    contents[cursor]=c;
    return *this;
}
inline Screen& Screen::set(pos r,pos col,char ch){
    contents[r*width+col]=ch;
    return *this;
}

class Window_mgr
{
public:
    using SreenIndex=std::vector<Screen>::size_type;
    void clear(SreenIndex);

    //向窗口添加一个Screen,返回它的编号
    SreenIndex addScreen(const Screen&);

private:

    std::vector<Screen> screens{Screen(24,80,' ')};
};

//函数的返回类型通常出现在函数名之前，因此当成员函数定义在类的外部时，
// 返回类型中使用的名字都位于类的作用域之外，此时返回类型必须指明它是哪个类
Window_mgr::SreenIndex
Window_mgr::addScreen(const Screen &s) {
    screens.push_back(s);
    return screens.size()-1;
}

void Window_mgr::clear(SreenIndex i) {
    Screen &s=screens[i];
    s.contents=std::string(s.height*s.width,' ');
}

//每个类定义了唯一的类型，对两个类来说，即使他们的成员完全一样，这两个类也是不同的类型
struct First{
    int memi;
    int getMem();
};

struct Second{
    int memi;
    int getMem();
};

class Link_screen {
    Screen window;
    Link_screen *next;
    Link_screen *prev;
};

//友元声明的作用是影响访问权限，它本身并非普通意义上的声明！！！！
struct  X{
    friend void f(){}
//    X() {f();}  //错误：f还没有被声明
    void g();
    void h();
};

void X::g(){return f();};   //错误：f还没有被声明
void f();

void X::h() {return f();}   //正确，现在f的声明在作用域中


typedef double Money;
std::string bal;
class Account{
public:
    //当编译器看到balance函数声明语句时，它将在Account类范围内寻找Money声明
    //编译器只考虑Account中在使用Money前出现的声明，因为没找到匹配的成员，
    //所以编译器会接着到Account的外层域中查找
    Money balance(){return bal;}

private:
    Money bal;
};

int main()
{
    Screen myscreen;
    char ch=myscreen.get();
    ch=myscreen.get(0,0);
    First obj1;
//    Second obj2=obj1;   //obj1和obj2类型不同
}