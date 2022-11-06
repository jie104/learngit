//3.2.2
#include <iostream>


int main()
{
    if(false){
        std::string line;
        std::getline(std::cin,line);
        std::cout << line << std::endl;

        std::cin >>line;
        std::cout << line << std::endl;
    }

    std::string c1;
    std::string c2;
    std::cin >> c1 >> c2;

    if (c1!=c2){
        if (c1 >c2){
            std::cout << "c1 is bigger than c2" << std::endl;
        }
        if (c1.size() > c2.size()){
            std::cout << "c1 is longer than c2!!!" << std::endl;
        }
    }

    std::string c3;
    std::string c_sum;
    while (std::cin >> c3 || c3.size()==0){
        c_sum+=c3+' ';
    }
    std::cout << c_sum << std::endl;

}