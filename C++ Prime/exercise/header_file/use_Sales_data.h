#ifndef _USE_SALES_DATA_
#define _USE_SALES_DATA_


#include <iostream>
#include <string>

struct Sales_data{
    int operator+(Sales_data data1){
        if (bookNo==data1.bookNo){
            unsigned totalCnt=units_sold+data1.units_sold;
            double totalRenvenue=revenue+data1.revenue;
            std::cout << data1.bookNo << " " << totalCnt
                      << " " << totalRenvenue << " ";
            if (totalCnt!=0)
                std::cout << totalRenvenue/totalCnt << std::endl;
            else
                std::cout << " (no sales)" << std::endl;
            return 0;
        }else{
            std::cerr << "Data must refer to the same ISBN" 
                      << std::endl;
            return -1;
        }
    }

    std::string bookNo;
    unsigned units_sold=0;
    double revenue=0.0;

};
#endif