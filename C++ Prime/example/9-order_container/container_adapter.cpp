//
// Created by zxj on 2023/1/5.
//

#include <iostream>
#include <stack>
#include <deque>
#include <vector>

int main(){
    std::deque<int> deq={1,2,3,3,4,5};
    std::stack<int> stk(deq);
    std::cout << "size: " << stk.size() << std::endl;
    int size=stk.size();
    for (int i=0;i<size;i++){
        std::cout << stk.top() << std::endl;
        stk.pop();
    }

    //vector上实现得空栈
    std::stack<std::string,std::vector<std::string>> str_stk;
    //str_stk2在vector上实现，初始化时保存svec得拷贝
    std::vector<std::string> svec;
    std::stack<std::string,std::vector<std::string>> str_stk2(svec);

    //栈适配器
    std::stack<int> intStack;   //空栈
    for (size_t ix=0;ix!=10;++ix)
        intStack.push(ix);
    while (!intStack.empty()){
        int value=intStack.top();
        intStack.pop();
    }
}

