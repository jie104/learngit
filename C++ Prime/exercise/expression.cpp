#include <iostream>
#include <vector>
#include <string>

int main()
{
//exercise 4.9
    const char *cp="Hello World" ;
    if (cp && *cp){
        std::cout << "yes" << std::endl;
    }

//exercise 4.10
    int n;
    std::vector<int> v;
    while (n!=42 && std::cin >> n){
        v.emplace_back(n);
    }

//exercise 4.11
    int a,b,c,d;
    if (a >b && b>c && c>d){
        std::cout << "a>b>c>d" << std::endl;
    }


//exercise 4.20
//    std::vector<std::string>::iterator iter;
//    *iter++;
//    (*iter)++;  //解引用后是string类型，不能++
//    *iter.empty();  //.运算优先级比解引用优先级高
//    iter->empty();  //判断是否是空字符
//    ++*iter;    //解引用后是string类型，不能++
//    iter++->empty();
//
//
//    std::string s2="adadaa";
    
//exercise 4.21
    std::vector<int> v1={1,2,3,4,5,6,7,8,9,120};
    for (auto it=v1.begin();it!=v1.end();++it){
        (*it%2!=0)? *it=2*(*it):*it;

    }
    for (const auto& x:v1){
        std::cout << x << std::endl;
    }

//exercise 4.22
    int grade;
    //mode1 
    std::string finalgrade=(grade >90) ? "high pass":
    (grade <75 && grade >60) ? "low pass" :
    (grade <60)? "pass" : "fail";
    //mode2
    if (grade<90){
        finalgrade="high pass";
    
    }else if(grade >60 && grade <75){
        finalgrade="low pass";
    }else if(grade <60){
        finalgrade="pass";
    }else{
        finalgrade="fail";
    }




}