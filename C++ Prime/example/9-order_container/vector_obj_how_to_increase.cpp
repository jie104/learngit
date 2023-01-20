//
// Created by zxj on 2023/1/4.
//

#include <iostream>
#include <vector>

int main()
{
    std::vector<int> ivec;
    std::cout << "ivec: size: " << ivec.size()
              << " capacity: " << ivec.capacity() << std::endl;

    //向ivec添加24个元素
    for (std::vector<int>::size_type ix=0;ix!=24;++ix){
        ivec.push_back(ix);
    }
    std::cout << "ivec: size: " << ivec.size()
              << " capacity: " << ivec.capacity() << std::endl;

    //预分配额外空间
    ivec.reserve(50);   //将capacity至少设定为50，可能会更大
    std::cout << "ivec: size: " << ivec.size()
              << " capacity: " << ivec.capacity() << std::endl;

    //用光预留空间
    while (ivec.size()!=ivec.capacity())
        ivec.push_back(0);
    std::cout << "ivec: size: " << ivec.size()
              << " capacity: " << ivec.capacity() << std::endl;

    ivec.push_back(42);
    std::cout << "ivec: size: " << ivec.size()
              << " capacity: " << ivec.capacity() << std::endl;

    //shrink_to_fit要求vector将多余空间退还
    ivec.shrink_to_fit();   //要求归还内存
    //调用shrink_to_fit只是一个请求，标准库并不保证退还内存
    std::cout << "ivec: size: " << ivec.size()
              << " capacity: " << ivec.capacity() << std::endl;

    std::vector<int> v={1,2,3};
    v.clear();
    v.resize(10);
    for (auto &x:v){
        std::cout << "v: " << x << std::endl;
    }


}

