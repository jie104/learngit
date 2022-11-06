#include <iostream>
#include <vector>

int main()
{
#ifdef _INBUILT_ARRAY2_
//exercise 3.35
    int a[4]={1,2,3,4};
    int *beg=std::begin(a); //指向数组的首元素
    int *end=std::end(a);   //指向数组尾元素的下一个元素
    for (int i=0;i < end-beg;i++){
        std::cout << a[i] << std::endl;
    }

    for (auto ptr=beg;ptr<end;ptr++){
        *ptr=0;
    }
    for (int i=0;i < end-beg;i++){
        std::cout << a[i] << std::endl;
    }
    

//exercise3.36
    int b[4]={};
    int *beg_b=std::begin(a);
    int *end_b=std::end(a);
    auto b_size=end_b-beg_b;
    auto a_size=end-beg;
    if (a_size == b_size){
        for (int j=0;j<end-beg;j++){
            if (a[j] != b[j]){
                std::cout << "array a!=b" << std::endl;
                break;
            }
        }
        std::cout << "array a==b" << std::endl;
    }else{
        std::cout << "array a!=b" << std::endl;
    }
        
    std::vector<int> c={1,2,3,4};
    std::vector<int> d={1,2,3,4};
    if (c==d){
        std::cout << "array c==d" << std::endl;
    }else{
        std::cout << "array c!=d" << std::endl;
    }
    
//exercise 3.37
    const char ca[]={'h','e','l','l','o'};
    const char *cp=ca;
    while (*cp){
        std::cout << *cp << std::endl;
        ++cp;
    }
#endif

//exercise 3.40
    char s[]="adsasafa";
    char s1[]="fczvsdv";
    // char s2[]=s1;
    const char *cp=s;
    while (*cp){
        std::cout << *cp << std::endl;
        ++cp;
    }
}