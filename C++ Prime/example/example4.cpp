#include <iostream>
#include <vector>

struct point{
    point(double a,double b):x(a),y(b){
    }
    double x,y;

};


int main()
{
    std::vector<int> nums;
    // nums.clear();
    std::cout << "size: " << nums.size() << std::endl;
    std::cout << "max_size: " << nums.max_size() << std::endl;
    std::cout << "capacity: " << nums.capacity() << std::endl;  //2的指数幂增加
    std::cout << "******************************************************************************" << std::endl;


    // nums.reserve(28);
    for (int i=0;i< 31;i++){
        nums.emplace_back();
        nums.back()=i;
        
        std::cout << "size: " << nums.size() << std::endl;
        std::cout << "max_size: " << nums.max_size() << std::endl;
        std::cout << "capacity: " << nums.capacity() << std::endl;  //2的指数幂增加
        std::cout << std::endl;
    }
    nums.resize(39);    //将当前vector前28个元素设置为0

    for (int i=0;i< 39;i++){
        std::cout << nums[i] << std::endl;
        std::cout << nums.size() << std::endl;
    }

    std::cout << "*********************************************************************************" << std::endl;
    nums.clear();
    std::cout << "size: " << nums.size() << std::endl;
    std::cout << "max_size: " << nums.max_size() << std::endl;
    std::cout << "capacity: " << nums.capacity() << std::endl;  //2的指数幂增加
    std::cout << std::endl;

}