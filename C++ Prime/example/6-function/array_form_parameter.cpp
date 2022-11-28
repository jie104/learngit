//
// Created by zxj on 2022/11/20.
//

#include <iostream>

void print(const int[]);
void print(const int *,const int *);
void print(const int ia[],size_t size);
void print(const int (&arr)[10]);


int main()
{
    int i=0;
    int cp[2]={1,2};
    int arr[10]={1,2,3,4,5,6,7,8,9,10};

    print(cp);
    print(&i);
    print(std::begin(cp),std::end(cp));
    print(cp,std::end(cp)-std::begin(cp));
    print(arr);

}

void print(const int *array){
    if (array){
        while (*array)
            std::cout << *array++ << std::endl;
    }
}

void print(const int *beg,const int *end){
    //输出beg到end之间（不含end）的所有元素
    while (beg!=end)
        std::cout << *beg++ << std::endl;
}

void print(const int ia[],size_t size){
    for (size_t i=0;i!=size;++i){
        std::cout << ia[i] << std::endl;
    }
}

//数组引用形参,arr是具有10个整数的整型数组引用
void print(const int (&arr)[10]){
    for (auto elem:arr)
        std::cout << elem << std::endl;
}