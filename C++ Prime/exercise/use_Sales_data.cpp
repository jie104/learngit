#include <iostream>
#include <string>
#include "header_file/use_Sales_data.h"


int main()
{
    Sales_data data1,data2;

    double price=0;

    std::cin >> data1.bookNo >> data1.units_sold >> price;
    //计算销售收入
    data1.revenue= data1.units_sold*price;

    std::cin >> data2.bookNo >> data2.units_sold >> price;
    data2.revenue=data2.units_sold*price;
    data1+data2;
}