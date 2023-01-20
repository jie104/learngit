//
// Created by zxj on 2022/12/20.
//
#include <iostream>
#include <fstream>
#include <string>

int main(int argc,char **argv)
{
    std::string ifile="myfile";
    std::ifstream in(ifile); //构造一个ifstream并打开文件
    std::ofstream out;  //输出文件流未与任何文件关联
    out.open(ifile+".copy");    //打开指定文件

    if(out){    //检查文件是否打开成功
                //open成功，我们可以使用文件
    }

    //一旦一个文件流已经打开，它就保持与对应文件的关联；
    //实际上，对一个已经打开的文件流调用open会失败，并导致failbit被置位
    //为了将文件流关联到另一个文件，首先必须关闭已经关联的文件
    //一旦文件成功哦in关闭，我们就可以打开新的文件
    in.close(); //关闭文件
    in.open(ifile+"2"); //打开另一个文件

    for (auto p=argv+1;p!=argv+argc;++p){
        std::ifstream input(*p);    //创建输出并打开文件
        if (input){

        }else{
            std::cerr << "could not open: " +std::string(*p);
        }
    }

}