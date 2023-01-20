//
// Created by zxj on 2023/2/1.
//

#include <iostream>
#include <set>
#include <map>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <utility>


struct Sale_data{
    int isbn;

};

bool compareIsbn(const Sale_data &lhs,const Sale_data &rhs){
    return lhs.isbn< rhs.isbn;
}

//创建pair对象的函数
std::pair<std::string,int> process(std::vector<std::string> &v){
    if (!v.empty()){//可以对·返回值进行列表初始化
        return {v.back(),v.back().size()};
    }else{
        return std::pair<std::string,int>();
    }
}


int main()
{
    //定义关联容器
    //每个关联容器都定义了一个默认构造函数，创建了一个指定类型的空容器
    std::map<std::string,size_t> word_count;    //空容器
    //列表初始化
    std::set<std::string> exclude={"the","but","and","or","an"};

    //三个元素：authors将姓映射为名
    std::map<std::string,std::string> authors={{"Joyce","James"},
                                               {"Austen","Jane"},
                                               {"Dickens","Charles"}};

    //初始化multimap或multiset
    std::vector<int> ivec;
    for (std::vector<int>::size_type i=0;i!=10;++i){
        ivec.push_back(i);
        ivec.push_back(i);
    }

    //iset包含来自ivec的不重复元素；miset包含所有20个元素
    std::set<int> iset(ivec.cbegin(),ivec.cend());
    std::multiset<int> miset(ivec.cbegin(),ivec.cend());
    std::cout << ivec.size() << std::endl;
    std::cout << iset.size() << std::endl;
    std::cout << miset.size() << std::endl;

    for (auto &x:ivec){
        std::cout << "ivec: " << x << std::endl;
    }

    for (auto &x:iset){
        std::cout << "iset: " << x << std::endl;
    }

    for (auto &x:miset){
        std::cout << "miset: " << x << std::endl;
    }

    auto tf=Eigen::Translation2d(Eigen::Vector2d::Zero())*Eigen::Rotation2Dd(0);
    std::cout << "tf： " << tf.matrix() << std::endl;

    auto A=Eigen::Matrix2d::Identity();
    std::cout << "A: " << A << std::endl;

    //关键字类型的要求
    //为了指定使用自定义的操作，必须在定义关联容器类型时提供此操作的类型
    //用尖括号指出要定义哪种类型容器，自定义的操作类型必须在尖括号中紧跟着元素类型给出
    /***
        用compareIsbn来初始化bookstore对象，这表示当我们向bookstore添加
        元素时，通过调用compareIsbn来为这些元素排序
     */
    std::multiset<Sale_data, decltype(compareIsbn)*> bookstore(compareIsbn);
    for (int i=0;i<10;i++){
        Sale_data s;
        if (i%2){
            s.isbn=i;
        }else{
            s.isbn=-1*i;
        }
        bookstore.insert(s);
    }

    for(auto &x:bookstore){
        std::cout << "isbn: " << x.isbn << std::endl;
    }

//pair类型
    //pair的默认构造函数对数据成员进行值初始化
    std::pair<std::string,std::string> anon;
    std::pair<std::string,size_t > word_counts;
    std::pair<std::string,std::vector<int>> line;

    std::pair<std::string,std::string> author{"James","Joyce"};
    std::cout << "author.first: " << author.first << "author.second: "
            << author.second << std::endl;

    std::pair<std::string,std::string> author1("James111","Joyce111");
    std::cout << "author1.first: " << author1.first << "author1.second: "
              << author1.second << std::endl;

    /***
     * make_pair(v1,v2)
        返回一个用v1和v2初始化的pair.pair的类型从v1和v2的类型推断出来
     */
    auto p=std::make_pair("daa",4);
    std::cout << "p.first: " << p.first << " p.second: " << p.second << std::endl;

    std::pair<std::string,int> p1;
    std::pair<std::string,int> p2;
    p1 < p2;

}

