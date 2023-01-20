//
// Created by zxj on 2023/2/1.
//

#include <map>
#include <iostream>
#include <string>
#include <set>

int main()
{
    std::map<std::string,size_t> word_count;
    std::set<std::string> exclude={"The","But","And","Or","An","A",
                                   "the","but","and","or","an","a"};

    std::string word;
    while(std::cin >> word && word!="q"){
        if(exclude.find(word)==exclude.end()){//只对不在集合中单词出现统计次数
            ++word_count[word];
        }
    }
    for (const auto &w:word_count){
        std::cout << w.first << " occurs " << w.second
                  << ((w.second >1) ? " times" : " time") << std::endl;
    }


}

