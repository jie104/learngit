//
// Created by zxj on 2023/2/24.
//

#ifndef EXAMPLE_QUERYRESULT_H
#define EXAMPLE_QUERYRESULT_H
#include <map>
#include <vector>
#include <fstream>
#include <string>
#include <memory>
#include <set>
#include<iostream>
#include "StrBlob.h"
class QueryResult
{
friend std::ostream &print(std::ostream&,const QueryResult&);

public:
    using line_no=std::vector<std::string>::size_type;
    QueryResult(std::string s,
                std::shared_ptr<std::set<line_no>> p,
                std::shared_ptr<StrBlob> f):
        sought(s),lines(p),file(f){ }

    std::shared_ptr<StrBlob> file; //输入文件

private:
    std::string sought; //查询单词
    std::shared_ptr<std::set<line_no>> lines;   //出现的行号
};

std::string make_plural (size_t ctr, const std::string& word, const std::string& ending)
{
    return (ctr>1)?word+ending:word;
}

std::ostream  &print(std::ostream &os,const QueryResult &qr){
    os << qr.sought << " occurs " << qr.lines->size() << " "
        << make_plural(qr.lines->size(),"time","s") << std::endl;

    for (auto num:*qr.lines){
        os <<"\t(line " << num+1 << ") " << *(qr.file->data->begin()+num) << std::endl;   //TODO

    }

    return os;
}

#endif //EXAMPLE_QUERYRESULT_H
