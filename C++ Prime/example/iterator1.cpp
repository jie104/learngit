//迭代器运算
#include <iostream>
#include <vector>
#include <string>



int main()
{
#ifdef _ITERATOR_
    std::vector<int>::iterator it;  //it能读写vector<int>的元素
    std::string::iterator it2;  //it只能读写string对象的字符

    //it3只能读元素，不能写元素
    std::vector<int>::const_iterator it3;   
    std::string::const_iterator it4;    


    std::vector<int> v;
    const std::vector<int> cv;
    auto it1=v.begin();     
    auto it2=cv.begin();

    std::string s="asdsafasfafgaa";
    auto it=s.begin();
    it=it+2;
    auto it1=s.begin();
    it1=it1+4;
    std::cout << *it << std::endl;
    auto num=it-it1;    //num类型是difference_type
    std::cout << "it-it1: " << num << std::endl;

    if (it > it1){
        std::cout << "it >it1" << std::endl;
    }else{
        std::cout << "it <=it1" << std::endl;
    }
#endif

    std::vector<int> text={1,2,3,4,5,6,7,89,100};
    auto beg=text.begin(),end=text.end();
    auto mid=text.begin()+(end-beg)/2;  //初始状态下的中间点
    int sought=88;
    while (mid!=end && *mid!=sought){
        if (sought <*mid)
            end=mid;
        else    
            beg=mid+1;
        mid=beg+(end-beg)/2;    //新的中间点
    }
    if(*mid==sought){
        std::cout << "find obj" << std::endl;
    }else{
        std::cout << "can't find obj" << std::endl;
    }

}