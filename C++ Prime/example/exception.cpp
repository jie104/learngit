//
// Created by zxj on 2022/11/11.
//

#include <iostream>
#include <stdexcept>

int main()
{
#ifdef _EXCEPTION_1111
    int item1,item2;
    std::cin >> item1 >> item2;
    if(item1!=item2){
        throw std::runtime_error("Dta must refer to same ISBN");
    }else{
        std::cout << item1+item2 << std::endl;
    }
#endif
    int item1,item2;

    while (std::cin >> item1 >> item2){
            try{
                std::cin >> item1;
            }catch (std::runtime_error err){
                std::cout << err.what()
                          << "\nTry Again? Enter y or n" << std::endl;

            }
            char c;
            std::cin >> c;
            if (!std::cin || c=='n'){
                break;
            }

    }



}
