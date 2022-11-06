#include <iostream>
#include <string>
#include <vector>
#include <ctype.h>

#define _STRING_
int main()
{
#ifdef _STRING1_

    std::string s5="hiya";   //拷贝u初始化
    std::string s6("hiya"); //拷贝初始化
    std::string s7(10,'c'); //直接初始化，s7内容是ccccccc
    
//读写string对象
//执行读写操作时，string对象会自动忽略开头的空白(空字符、换行符、制表符等)，
//并从第一个字符算起，直到遇见下一处空白为止
    if(false){
        std::string s;  //空字符串
        std::cin >> s;  //将string对象读入s，遇到空白停止
        std::cout << s << std::endl;    //输出s

        std::string s1,s2;
        std::cin >> s1 >> s2;   //将第一个输入读到s1中，第二个输入读到s2中
        std::cout << s1 << s2 << std::endl;
    }
    if (false)
    {
        std::cout << "开始读取操作" << std::endl;
        std::string word;
        while (std::cin >> word)    //反复读取，直到到达文件末尾
            std::cout << word << std::endl; //逐个输出单词，每个单词后面紧跟一个换行
    }

    if(false)
    {
        std::string line;
        //每次读入一行，直至到达文件末尾
        while (std::getline(std::cin,line))
            if (!line.size() >80)
                std::cout << line << std::endl;
    }

    //由于size函数返回的是一个无符号整型数，因此切记，如果表达式中混用了带符号
    //和无符号将产生意想不到的结果
    auto l=s5.size();
    std::vector<int> v1;
    auto l1=v1.size();



//字面值和string对象可以相加

    // if(false)
    // {

    //     std::string s1="hello",s2="world";
    //     std::string s3=s1+", " +s2+'\n';
    //     std::string s4=s1+", "; //正确：把string对象和一个字面值相加
    //     std::string s5="hello" +", ";   //错误，两个运算对象都不是string
    //     //正确：每个加法运算符都有一个运算对象是string
    //     std::string s6=s1+", "+ "world";
    //     std::string s7="hello" + ", "+s2;   //错误：不能将字面值直接相加

    // }
    if (false)
    {
    std::string str("some string");
    for (auto c:str){
        std::cout << c << std::endl;
    }
    }
    std::string s("Hello World!!!");
    decltype(s.size()) punct_cnt=0;
    //统计s中的标点符号的数量
    for (auto c:s){
        if (ispunct(c)){
            ++punct_cnt;
        }
    }
    std::cout << punct_cnt << " punctuation chahracters in " << s << std::endl;




    std::string s("Hello World!!!");
    for (auto&c:s){
        c=std::toupper(c);
    
    }
    std::cout << s << std::endl;

//任何表达式，只要他的值是一个整数，就能作为索引，不过，如果某个索引是带符号类型的表达式的值将
//自动转换成由string::size_type表达的无符号类型
    std::string s("some string!!!");

    for (decltype(s.size()) index=0;
        index!=s.size() && !isspace(s[index]);++index)
            s[index]=toupper(s[index]); //将当前字符改成大写形式
    std::cout << s << std::endl;

#endif


    const std::string hexdigits="0123456789ABCDEF";
    std::cout << "      Enter a series of numbers between 0 and 15" 
              << " separated by spaces. Hit ENTER when finished: "
              << std::endl;
    std::string result;     //用于保存十六进制的字符串
    std::string::size_type n;   
    while (std::cin >> n){
        if (n < hexdigits.size())
            result+=hexdigits[n];
        
    }
    std::cout << "Your hex number is: " << result << std::endl;


}
