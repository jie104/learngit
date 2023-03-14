//
// Created by zxj on 2023/2/24.
//

#ifndef EXAMPLE_TEXTQUERY_H
#define EXAMPLE_TEXTQUERY_H
#include "QueryResult.h"
#include "StrBlob.h"

class QueryResult;
class TextQuery{
public:
    using line_no=std::vector<std::string>::size_type;
    TextQuery(std::ifstream&);
    QueryResult query(const std::string&) const;

private:
    std::shared_ptr<StrBlob> file;
//    std::shared_ptr<std::vector<std::string>> file; //输入文件
    //每个单词到它所在行的集合的映射
    std::map<std::string,std::shared_ptr<std::set<line_no>>> wm;
};


#endif //EXAMPLE_TEXTQUERY_H
