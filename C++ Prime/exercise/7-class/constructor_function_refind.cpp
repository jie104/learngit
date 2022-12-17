//
// Created by zxj on 2022/12/12.
//

#include <iostream>
#include <string>
#include <vector>

struct X{
    X(int i,int j):base(i),rem(base&%j) {}
    int base,rem;
};

class Book
{
public:
    Book(std::string bookNO,std::string author,std::string isbn)
        :bookNo_(bookNO),author_(author),isbn_(isbn){ }
    Book(): Book("12","jie","2345465677"){}
private:
    std::string bookNo_;
    std::string author_;
    std::string isbn_;
};
