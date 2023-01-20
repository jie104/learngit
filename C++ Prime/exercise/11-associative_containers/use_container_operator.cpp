//
// Created by zxj on 2023/2/10.
//

#include <iostream>
#include <map>
#include <string>
#include <vector>

int main()
{
    //exercise 11.21
#ifdef _ELEVEN_TWENTY_ONE_
    std::map<std::string,size_t > word_count;
    std::string word;
    while (std::cin >> word && word!="q") {
        ++word_count.insert({word,0}).first->second;
    }

    for(auto &x:word_count){
        std::cout << "first: " << x.first << " second: " << x.second  << std::endl;
    }
#endif

#ifdef _ELEVEN_TWENTY_TWO_
    //exercise 11.22
    std::map<std::string,std::vector<int>> word_count1;
    auto imap=word_count1.insert({"a",{1,2}});

    //exercise 11.28
    std::map<std::string,std::vector<int>> imap1={{"as",{1,2}},{"ew",{35,5}}};
    auto iter=imap1.find("as");

#endif

#ifdef _ELEVEN_TWENTY_EIGHT_
    //exercise 11.31
    std::multimap<std::string,std::string> authors;
    authors={{"r","as"},{"a","dwq"},{"f","qdq"},{"w","rwe"}};
    for (auto &x: authors){
        std::cout << "first: " << x.first << " second: " << x.second << std::endl;
    }
    std::string name;
    while (std::cin >> name && name!="q"){
        if (authors.find(name)!=authors.end()){
            authors.erase(name);
        }else{
            std::cout << "can not find " << name << std::endl;
        }
    }
    for (auto &x: authors){
        std::cout << "first: " << x.first << " second: " << x.second << std::endl;
    }
#endif


}

