#include <string>
#include <iostream>

//我们认定某个类型是迭代器当且仅当他支持一套操作，这套操作使我们能访问容器的元素或从某个元素移动到另外一个元素
int main()
{
    std::string s("some string");
    if (s.begin() !=s.end()){   //确保s非空
        auto it=s.begin();  //it表示s的第一个字符
        *it=toupper(*it);
    }
    std::cout << s << std::endl;

//迭代器从一个元素移动到另一个元素
    for (auto it=s.begin();it!=s.end() && !isspace(*it);++it)
        *it=toupper(*it);   //将当前字符改成大写形式
    std::cout << s << std::endl;
}