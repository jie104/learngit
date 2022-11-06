#include <iostream>


int main()
{
    unsigned vowelCnt=0;
    unsigned aCnt=0,eCnt=0,iCnt=0,oCnt=0,uCnt=0;
    char ch;
//表达式和某个case标签的值匹配成功，程序从该标签之后的第一条语句开始执行
//直到遇到了switch的结尾或遇到一条break语句为止
    while (std::cin >> ch && ch !='q'){
        //如果ch是元音字母，将其对应计数加1
        switch(ch){
            case 'a':
                ++aCnt;
                break;
            case 'e':
                ++eCnt;
                std::cout << "e" << std::endl;
                break;
            case 'i':
                ++iCnt;
                break;
            case 'o':
                ++oCnt;
                break;
            case 'u':
                ++uCnt;
                break;
            //如果没有任何一个case标签能匹配上switch表达式语句的值，
            //程序将执行default标签后面的语句，此时default标签位置随意
            default:    
                ++vowelCnt;
                std::cout << "begin to default" << std::endl;
                break;

        }
    }
    //输出结果
    std::cout << "Number of vowel a: \t" << aCnt << '\n'
              << "Number of vowel e: \t" << eCnt << '\n'
              << "Number of vowel i: \t" << iCnt << '\n'
              << "Number of vowel o: \t" << oCnt << '\n'
              << "Number of vowel a: \t" << uCnt << std::endl;

//case1关键字和他对应的值一起被称为case标签，case标签必须是整型常量表达式
    char ch1='s';
    int ival=42;
    switch(ch){
        // case 3.14:  //错误：case标签不是一个整数
        // case ival:  //错误：case标签2不是一个常量
    }

    switch(ch)
    {
        case 'a':
        case 'e':
        case 'i':
        case 'o':
        case 'u':
            ++vowelCnt;
            break;
    }
}