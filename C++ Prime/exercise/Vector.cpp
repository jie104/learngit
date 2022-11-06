//exercise3.3.1
#include <iostream>
#include <vector>
#include <string>


template <class Type>
void printVectorElement(const std::vector<Type>& v){
    for (int i=0;i< v.size();i++){
        std::cout << v[i] << std::endl;
    }
}


int main()
{
    std::vector<int> v1;
    std::vector<int> v2(10);    //初始化10个元素，均为0
    std::vector<int> v3(10,42); //初始化10个元素，均为42
    std::vector<int> v4{10};    //初始化列表，大括号，即初始化一个元素
    std::vector<int> v5{10,42};
//确认无法执行列表初始化后，编译器会尝试用默认值初始化vector对象
    std::vector<std::string> v6{10};
    std::vector<std::string> v7{10,"hi"};

#ifdef _VECTOR_
    std::cout << "v2: " << std::endl;
    printVectorElement(v2);
    std::cout << "v3: " << std::endl;
    printVectorElement(v3);
    std::cout << "v4: " << std::endl;
    printVectorElement(v4);


    std::cout << "v7: " << std::endl;
    printVectorElement(v7);

#endif

    std::string word;
    std::vector<std::string> text;  //空vector对象
    while (std::cin >> word){
        text.push_back(word);   //把word添加到text后面
    }



}