#include <iostream>
#include <eigen3/Eigen/Core>


int main()
{
    Eigen::Vector2d a(0,0);
    std::cout << a.transpose() << a[0] << a[1] << std::endl;
}