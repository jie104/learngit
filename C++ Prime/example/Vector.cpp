#include <iostream>
#include <string>
#include <vector>

template <class type>
void printVectorElement(std::vector<type>& v){
    for (int i=0;i< v.size();i++){
        std::cout << v[i] << std::endl;
    }
    std::cout << "size: " << v.size() << std::endl;
    std::cout << "capacity: " << v.capacity() << std::endl;
    std::cout << "max_size: " << v.max_size() << std::endl;

}

int main()
{
//创建指定数量的元素
    std::vector<int> ivec(10,-1);//十个int类型元素，每个都被初始化为-1
    std::vector<std::string> svec(10,"hi");
    printVectorElement(svec);

    std::vector<int> v1;
    v1.resize(10,0);
    printVectorElement(v1);

    std::vector<int> v2(10);
    std::vector<std::string> v3(10);
    printVectorElement(v2);
    printVectorElement(v3);

    std::vector<int> v4(10);    //v4有10个元素，每个元素都是0
    std::vector<int> v5(10);    //v5有1个元素，该元素是10

    std::vector<int> v6(10,1);  //v6有10个元素，每个元素值都是1
    std::vector<int> v7{10,1};  //v4有2个值，值分别是10和1


    




}