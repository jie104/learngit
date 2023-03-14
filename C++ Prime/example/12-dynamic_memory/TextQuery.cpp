//
// Created by zxj on 2023/2/24.
//

#include "head_file/TextQuery.h"
#include <sstream>
#include <memory>

//读取输入文件并建立单词到行号的映射
TextQuery::TextQuery(std::ifstream &is):file(new StrBlob) {
    std::string text;
    while (std::getline(is,text)){
        file->push_back(text);
        int n=file->size()-1;
        std::istringstream line(text);  //将行文本分解为单词
        std::string word;
        while (line >> word){
            auto &lines=wm[word];
            if(!lines)
                lines.reset(new std::set<line_no>);
            lines->insert(n);
        }
    }
}

QueryResult TextQuery::query(const std::string &sought) const{
    //如果未找到sought，将返回一个指向此set的指针
    static std::shared_ptr<std::set<line_no>> nodata(new std::set<line_no>);
    //使用find而不是下标运算符查找单词，避免将单词添加到wm
    auto loc=wm.find(sought);
    if (loc==wm.end())
        return QueryResult(sought,nodata,file); //未找到
    else
        return QueryResult(sought,loc->second,file);
}
