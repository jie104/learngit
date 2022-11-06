#include <iostream>
#include <map>

int main()
{
    std::map<double,double> A;
    A.insert(std::pair<double,double>(2,1));
    A.insert(std::pair<double,double>(3,1));

    for (auto&a: A){
        std::cout << a.first << " " << a.second << std::endl;
    }


}

