//
// Created by zxj on 2022/11/25.
//

#include <iostream>
#include <vector>

int &get(int (&array)[10],int index){
    return array[index];
}

void printVectorElement(std::vector<int>& v){
    auto v1=v;
    if (v1.size()!=0){
        int n=v1.size()-1;
        std::cout << v1[n] << std::endl;
        v1.pop_back();
        printVectorElement(v1);
    }else{
        return;
    }
}

int main()
{
    int ia[10];
    std::vector<int> v={1,2,3,456};
    for (int i=0;i!=10;++i){
        get(ia,i)=i;
        std::cout << get(ia,i) << std::endl;
    }
    printVectorElement(v);
}
