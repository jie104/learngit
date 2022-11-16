//
// Created by zxj on 2022/11/11.
//

#include <iostream>
#include <string>

int main()
{
    std::string buf;
    while (std::cin >> buf && !buf.empty()){
        switch(buf[0]){
            case '-':
                for (auto it=buf.begin()+1;it!=buf.end();++it){
                    if (*it==' ')
                        break;
                }
                break;
            case '+':
                //......
                break;
        }
    }
}