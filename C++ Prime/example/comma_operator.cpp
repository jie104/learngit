#include <iostream>
#include <vector>

int main()
{
    std::vector<int> ivec={1,2,3,4};
    std::vector<int>::size_type cnt=ivec.size();
    for (std::vector<int>::size_type ix=0;
        ix!=ivec.size();++ix,--cnt){
            ivec[ix]=cnt;
        }
    int i=0,j=1;

//逗号运算符，首先对左侧表达式求值，然后将求值结果丢弃调掉；逗号运算符真正的结果是右侧表达式的值
//如果右侧运算对象是左值，那么最终对象的求值结果也是左值
   std::cout << (1,2) << std::endl;     //2
    std::cout << (++i,++j) << std::endl;       //2
}