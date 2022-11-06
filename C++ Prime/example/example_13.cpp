#include <iostream>
#include <vector>

int main()
{
    std::vector<double> d;
    std::cout << "size: "  << d.size() << std::endl;
    for (int i=0;i<2;i++){
        d.reserve(d.size()+2);
        std::cout << "capacity: "  << d.capacity() << std::endl;
        for (int j=0;j<2;j++){
            d.push_back(1);
        }

        for (int k=0;k<d.size();k++){
            std::cout << d[0] << std::endl;
        }
        std::cout << "*****************************************" << std::endl;

    }
}