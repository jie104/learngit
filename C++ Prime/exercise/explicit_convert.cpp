#include <iostream>
#include <string>
// #define _EXPLICIT_CONVERT_

int main()
{
#ifdef _EXPLICIT_CONVERT_
//exercise 4.36
    int i;
    double d;
    i=i*static_cast<int>(d);
#endif
//exercise 4.37
    int i;
    double d;
    const std::string *ps;
    char *pc;
    // void *pv;
    auto pd=const_cast<std::string*>(ps);
    void* pv=static_cast<void*>(pd);

    i=static_cast<int>(*pc);
    pv=static_cast<void*>(&d);
    pc=static_cast<char*>(pv);



}