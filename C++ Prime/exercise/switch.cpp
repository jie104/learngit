//
// Created by zxj on 2022/11/9.
//

#include <iostream>

unsigned get_bufCnt(){
    return 1;
}
int main()
{
//exercise 5.9、5.10
#ifdef _SWITCH_H
    char ch;
    int aCnt=0,oCnt=0,iCnt=0,eCnt=0,uCnt=0;
    while (std::cin >> ch && ch!='q'){
        ch=tolower(ch);
        if(ch=='a'){
            ++aCnt;
        }else if (ch=='o'){
            ++oCnt;
        }else if (ch=='i'){
            ++iCnt;
        }else if (ch=='e'){
            ++eCnt;
        }else if (ch=='u'){
            ++uCnt;
        }
    }
    std::cout << "aCnt: " << aCnt << std::endl;
    std::cout << "oCnt: " << oCnt << std::endl;
    std::cout << "iCnt: " << iCnt << std::endl;
    std::cout << "eCnt: " << eCnt << std::endl;
    std::cout << "uCnt: " << uCnt << std::endl;

#endif
    const unsigned ival=512,jval=1024,kval=4096;
    unsigned bufsize;
    unsigned swt=get_bufCnt();
    //case关键字和它对应的值一起被称为case标签，case标签必须是整型常量表达式
    switch(swt){
        case ival:
            bufsize=ival*sizeof(int);
            break;
        case jval:
            bufsize=ival*sizeof(int);
        case kval:
            bufsize=kval*sizeof(int);
            break;
    }
}