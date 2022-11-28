//
// Created by zxj on 2022/11/22.
//

#include <iostream>
#include <string>

int compareInt(const int n,const int *k);
void exchangeIntPtr(int *,int *);
void print(const int (&ia)[3]);


int main(int argc,char **argv)
{
    int a=3,b=4;
    int ss[3]={1,2,3};
    std::cout << compareInt(a,&b) << std::endl;

    exchangeIntPtr(&a,&b);
    std::cout << "a: " << a << " b: " << b << std::endl;

    print(ss);

    std::cout << std::string(argv[1])+std::string(argv[2]) << std::endl;

}

//exercise 6.21
int compareInt(const int n,const int  *k){
    if (n > *k){
        return n;
    }else{
        return *k;
    }
}

//exercise 6.22
void exchangeIntPtr(int *ptr1,int *ptr2){
    auto temp=*ptr2;
    *ptr2=*ptr1;
    *ptr1=temp;


}

void print(const int (&ia)[3]){
    for (size_t i=0;i!=10;++i)
        std::cout << ia[i] << std::endl;
}