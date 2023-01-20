//
// Created by zxj on 2023/1/14.
//

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

void fcn4(){
    size_t v1=42;
    //v1是一个非const变量的引用
    //可以通过f2中的引用来改变它
    auto f2=[&v1]{return ++v1;};
    v1=0;
//    auto j=f2();
    std::cout << "f2():" << f2() << std::endl;  //1
}


//可变lambda
//默认情况下，对于一个值被拷贝的列表，lambda不会改变其值。
//如果希望能改变一个被捕获的变脸的值，必须在参数列表首加上关键字mutable

void fcn3() {
    size_t v1 = 42;   //局部变量
    //f可以改变它所捕获的变量的值
    auto f = [v1]() mutable { return ++v1; };
    v1 = 0;
//    auto j = f(); //j为43
    std::cout << "f(): " << f() << std::endl;
}

std::string make_plural(size_t ctr,const std::string &word,
                        const std::string &ending){
    return (ctr >1)? word +ending:word;
}

void elimDups(std::vector<std::string> &words)
{
    //按字典排列words,以便查找单词
    std::sort(words.begin(),words.end());
    //unique重排输入范围ie，使得每个元素只出现一次
    //排列在范围的前部，返回不重复区域之后一个位置的迭代器
    auto end_unique=std::unique(words.begin(),words.end());

    //使用向量操作erase删除重复单词
    words.erase(end_unique,words.end());
}

bool isShorter(const std::string &s1,const std::string &s2){
    return s1.size()<s2.size();
}

//打印大于等于给定长度的单词
void biggies(std::vector<std::string> &words,
             std::vector<std::string>::size_type sz){
    elimDups(words);
    std::stable_sort(words.begin(),words.end(), isShorter);
    //获取一个迭代器，指向第一个满足size()>sz的元素
    //计算满足size>=sz的元素的数目
    //打印长度大于给定值的单词，每个单词后面接一个给定的空格

}


int main()
{
/***
    lambda表达式 [capture list](parameter list) -> return type {function body}
        capture list 捕获列表，是一个lambda所在函数中定义的局部变量的列表（通常为空）
        return type 返回类型
        parameter list 参数列表
        function body 函数体
 ***/

//    //lambda中忽略括号和参数列表等价于指定一个空参数列表
//    auto f=[]{return 42;};
//    //如果忽略返回类型，lambda根据函数体中的代码推断出返回类型
//    std::cout << f() << std::endl;  //返回int类型
//
//    //如果lambda的函数体包含任何单一return语句之外的内容，且未指定
//    //返回类型，则返回void
//    auto f1=[]{int k=0;return k;};
//    std::cout << f1() << std::endl;

    //向lambda传递参数
//    std::vector<std::string> words={"saas","rwrg","rwrg","hrnre","s","efw","rgewaadf"};
//    elimDups(words);
//    std::stable_sort(words.begin(),words.end(),
//                     [](const std::string &a,const std::string &b) {return a.size() < b.size();});
//
//    for (auto &x: words){
//        std::cout << x << std::endl;
//    }
//
//    int sz=3;
//    auto wc=std::find_if(words.begin(),words.end(),
//                         [sz](const std::string &a){return a.size()>=sz;});
//
//    auto count=words.end()-wc;
//    std::cout << count << " " << make_plural(count,"word","s")
//                << " of length " << sz << " or longer" << std::endl;
//
//    //打印长度大于等于给定值的单词，每个单词后面接一个空格
//    //捕获列表只用于局部非static变量，lambda可以直接使用局部static变量和
//    //它所在函数之外声明的名字
//    std::for_each(wc,words.end(),
//                  [](const std::string &s){std::cout << s << " ";});
//    std::cout << std::endl;

    fcn3();
    fcn4();

    //指定lambda返回类型
    std::vector<int> vi={-1,2,3,-4,54,5,-67,87};
//    std::transform(vi.begin(),vi.end(),vi.begin()+(vi.end()-vi.begin())/2+2,
//                   [](int i){return i <0?-i:i;});

    //错误：不能推断lambda的返回类型
    std::transform(vi.begin(),vi.end(),vi.begin(),
                   [](int i){if(i<0) return -i; else return i;});
    for (auto &x:vi){
        std::cout << "vi: " << x << std::endl;
    }

//默认情况下，如果一个lambda体包含return之外的任何语句，则编译器假定此lambda返回void
//编译器推断这个版本的lambda返回类型为void，但它返回一个int
// 需要为一个lambda定义返回类型时，必须使用尾置返回类型
    std::transform(vi.begin(),vi.end(),vi.begin(),
                   [](int i)->int
                   {if(i<0) return -i; else return i;});

}

