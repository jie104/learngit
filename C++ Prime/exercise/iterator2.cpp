//exercise 3.21
#include <iostream>
#include <vector>
#include <string>

template <class Type>
    void printVectorElement(std::vector<Type>& v){
        if (v.begin()!=v.end()){
            for (auto it=v.begin();it !=v.end();it++){
                std::cout << *it << std::endl;
            }
        }else{
            std::cout << "v have not elements!!!" << std::endl;
        }
    }


int main()
{
#ifdef _ITERATOR2_
    std::vector<int> v1;
    std::vector<int> v2(10);
    std::vector<int> v3(10,42);
    std::vector<int> v4{10};
    std::vector<int> v5{10,42};
    std::vector<std::string> v6{10};
    std::vector<std::string> v7{10,"hi"};
    std::cout << "print v1" << std::endl;
    printVectorElement(v1);
    std::cout << "print v2" << std::endl;
    printVectorElement(v2);
    std::cout << "print v3" << std::endl;
    printVectorElement(v3);
    std::cout << "print v4" << std::endl;
    printVectorElement(v4);
    std::cout << "print v5" << std::endl;
    printVectorElement(v5);
    std::cout << "print v6" << std::endl;
    printVectorElement(v6);
    std::cout << "print v7" << std::endl;
    printVectorElement(v7);

    // std::string text="dasfsfa,fasfa";
    // for (auto it=text.cbegin();
    //     it!=text.cend()&& !it->empty();++it){
    //         std::cout << *it << std::endl;
    //         *it=toupper(*it);
    //     }

//exercise 3.23
    std::vector<int> v(10,34);
    for (auto it=v.begin();it!=v.end() &&v.begin()!=v.end();
        it++){
        *it=2*(*it);
        std::cout << *it << std::endl;
    }


//exercise 3.24
    int n;
    std::vector<int> v1;
    while (std::cin >> n){
        v1.push_back(n);
    }

    // for (auto it=v1.begin();it!=v1.end()-1;++it){
    //     *it=*it+*(it+1);
    //     std::cout << *it << std::endl;
    // }

    for (auto it=v1.begin();it!=v1.end()-(v1.end()-v1.begin())/2;++it){
        auto move_num=it-v1.begin();
        auto it1=v1.end()-move_num-1;
        int first_last_sum=*it+*it1;
        std::cout << first_last_sum << std::endl;
    }
#endif

//exercise 3.25
    std::vector<unsigned> scroes(11,0); 
    unsigned grade;
    while (std::cin >> grade){
        if (grade <=100){
            auto it=scroes.begin()+grade/10;
            *it=*it+1;
        }    

    }
    printVectorElement(scroes);

    auto beg=scroes.begin();
    auto end=scroes.end();
}