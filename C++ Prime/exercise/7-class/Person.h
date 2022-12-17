//
// Created by zxj on 2022/12/4.
//

#ifndef EXERCISE_PERSON_H
#define EXERCISE_PERSON_H

#include <iostream>
#include <string>
class X;
class Y;

class Person
{
public:
    Person()=default;
    Person(std::string name,std::string adress):name_(name_),address_(address_){}
    std::string& getName() const {return name_;}
    std::string& getAddress() const {return  address_;}
private:
    std::string name_;
    std::string address_;
};

std::istream& read(std::istream& is,const Person& person){
    is >> "name: " >> person.getName() >> ",address: " >> person.getAddress();
    return is;
}

std::ostream& print(std::ostream& os,const Person& person){
    os << "name: " << person.getName() << ",address: " << person.getAddress();
    return os;
}

class X{
    Y *y_ptr;
};

class Y{
    X x;
};


#endif //EXERCISE_PERSON_H
