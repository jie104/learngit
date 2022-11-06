#include <iostream>
#include <vector>
#include <string>


int main()
{
#ifdef _EXPRESSION_
//左值和右值
//当对象被用作右值时候，用的是对象的值(内容)；
//当对象被用作左值时，用的是对象的身份（在内存中的位置）
//一个重要的原则是在需要右值的地方可以用左值来代替，但不能把右值当左值使用
    int *p;
    int x=1;
//使用关键字decltyp[e时，左值和右值也所不同。
//如果表达式的求值结果是左值，decltype作用于该表达式(不是变量)得到一个引用类型
    decltype(*p) a=x;
    decltype(&p) b;

//求值顺序
//对于那些没有指定执行顺序的运算符来说，如果表达式指向并修改了同一个对象，将会引发错误并产生未定义行为
    int i=0;
    std::cout << i << " " << ++i << std::endl;  //未定义

    int i=1024;
    int k=-1;   //k是-1024
    bool b=true;
    bool b2=-b;     //b2是true

//运算符%俗称“取余”或“取模”运算符，负责处理两个整数相除所得的余数
    int ival=42;
    double dval=3.14;
    ival%12;    //正确：结果是6
    // ival% dval; //错误：运算对象是浮点对象



//逻辑与逻辑或运算符
    std::vector<std::string> text;
    std::string s;
    int i=0;

    while (std::cin >> s){
        if (s=="q"){
            break;
        }
        i++;
        text.emplace_back(s);
        std::cout << i << std::endl;

    }

    for (const auto &s: text){
        std::cout << s;
        //遇到空字符或以句号结束的字符串进行换行
        if (s.empty() || s[s.size()-1]=='.'){
            std::cout << std::endl;
        }else{
            std::cout << " ";   //否则用空格隔开
        }
    }


    // std::string s1;
    // std::cin >> s1;
    // std::cout << s1;
//递增、递减运算符
    int i=0,j;
    j=++i;  //j=1,i=1:前置版本得到递增之后的值
    j=i++;  //j=1,j=2:后置版本得到递增之前的值


//在一条语句中混用解引用和递增运算符
    std::vector<int> v;
    auto pbeg=v.begin();
    //输出元素直到遇到第一个负值为止
    while (pbeg!=v.end() && *pbeg>=0){
        std::cout << *pbeg++ << std::endl;
    }


    while (pbeg!=v.end() && !isspace(*pbeg)){
        *pbeg=toupper(*pbeg++); //错误:该赋值语句未定义
    }

//成员访问运算符
    std::string s1="a string",*p=&s1;
    auto n=s1.size();   //运行string对象s1的size成员
    n=(*p).size();  //运行p所指对象的size成员
    n=p->size();     //等价于(*p).size()

//条件运算符
    int grade=30;
    std::string finalgrade=(grade <60) ? "fail": "pass";
    //条件运算符一般满足右结合律，意味着运算对象按照从右往左的顺序组合
    finalgrade =(grade >90) ? "high pass": (grade <60) ? "fail" :"pass";

//在输出表达式中使用条件运算符
    std::cout << ((grade <60) ? "fail" : "pass");   //输出pass或fail
    std::cout << (grade <60) ? "fail" : "pass";   //输出1或0
    std::cout << grade <60 ? "fail" : "pass";   //错误：试图比较cout和60
#endif

//移位运算符 << >>
    unsigned char bits=0233;
    unsigned long a=(bits << 24);  //bits提升成int类型，然后向左移动8位
    std::cout << a << std::endl;

//位反运算符~
    unsigned char bits_1=0227;
    std::cout << (int)bits_1 << std::endl;
    unsigned long b=~bits_1;
    std::cout << (unsigned long)b << std::endl;
    
//位与、位或、位异运算符
    unsigned char b1=0145;
    unsigned char b2=0257;
    b1 & b2;    //两个运算对象的对应位置都是1，则运算结果改位是1
    b1 | b2;
    b1 ^ b2;    //如果两个运算对象的对应位置有且只有一个为1，则运算结果中该位为1，·否则是0

//使用位运算符
    unsigned long quiz1=0;
    
}