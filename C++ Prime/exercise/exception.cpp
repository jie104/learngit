//
// Created by zxj on 2022/11/13.
//

#include <iostream>
#include <stdexcept>

int main()
{
    int num1,num2;
    while (std::cin >> num1 >> num2){
        try{
            std::cout << static_cast<double>(num1/num2) << std::endl;
        }catch(std::length_error err){
            std::cout << err.what()
                << "\nnum2 is zero" << std::endl;
            std::cout << "You plan to try again???"
                         "please enter y or n" << std::endl;
            char c;
            std::cin >> c;
            if (!std::cin && c=='n'){
                break;
            }
        }

    }

}
