#include <iostream>


int main()
{
#ifdef _INTERGRAL_PROMOTION_
//exercise 4.34
    float fval=1.0;
    int ival=2;
    char cval='c';
    double dval=fval+ival;
    dval=fval+ival;
    auto a=dval+ival*cval;
#endif

//exercise 4.35
    char cval;
    int ival;
    unsigned int ui;
    float fval;
    double dval;
    
}