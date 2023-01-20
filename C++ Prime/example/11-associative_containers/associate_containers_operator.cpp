//
// Created by zxj on 2023/2/4.
//
//#define _MAP_SUBSCRIPT_OPTIONS_
//#define _ELEMENT_VISIT_

#include <map>
#include <string>
#include <set>
#include <iostream>
#include<vector>
#include <fstream>
#include <exception>
#include <sstream>

//读入给定文件，建立起转换关系
std::map<std::string,std::string> buildMap(std::ifstream &map_file){
    std::map<std::string,std::string> trans_map;    //保存转化规则
    std::string key,value;
    //读取第一个单词存入key后，行中剩余内容存入value
    while(map_file >> key && std::getline(map_file,value)){
        if(value.size() >1){
            trans_map[key]=value.substr(1); //跳过前导空格
        }else{
            throw std::runtime_error(":no rule for "+key);
        }
    }
    return trans_map;
}

//生成转换文本
const std::string &transform(const std::string &s,
                             const std::map<std::string,std::string> &m){
    //实际的转换工作；此部分是程序的核心
    auto map_it=m.find(s);
    //如果单词在转换规则map中
    if(map_it !=m.cend()){
        return map_it->second;
    }else{
        return s;
    }
}

void word_transform(std::ifstream &map_file,std::ifstream &input){
    auto trans_map= buildMap(map_file); //保存转换规则
    std::string text;
    while (std::getline(input,text)){   //读取每一行输入
        std::istringstream stream(text);    //读取每个单词
//        std::cout << "stream: " << stream.str() << std::endl;
        std::string word;
        bool firstword=true;
        while (stream >>word){
            if (firstword)
                firstword= false;
            else
                std::cout << " ";
            std::cout << transform(word,trans_map);
        }
        std::cout << std::endl; //完成一行转换
    }
}

int main()
{
#ifdef _ASSOCIATE_CONTAINER_OPERATOR_
    //关联容器操作
    std::set<std::string>::value_type v1;   //v1是string
    std::set<std::string>::key_type v2; //v2是一个string
    std::map<std::string,int>::value_type v3;   //v3是一个pair<const string,int>
    std::map<std::string,int>::key_type v4; //v4是一个string
    std::map<std::string,int>::mapped_type v5;  //v5是一个int

    std::cout << v3.first <<std::endl;
    std::cout << v3.second << std::endl;

#endif

#ifdef _ASSOCIATE_CONTAINER_ITERATOR_
//关联容器迭代器
    //获得指向word_count中一个元素的迭代器
    std::map<std::string,size_t > word_count{{"dasa",1},{"erww",5}};
    auto map_it=word_count.begin();
    //*map_it是指向一个pair<const string,size_t>对象的引用
    std::cout << map_it->first << std::endl;
    std::cout << " " << map_it->second;
//    map_it->first="new key";    //错误：关键字是const
    ++map_it->second;   //正确：可以通过迭代器改变元素

    //set的迭代器是const
    std::set<int> iset={0,1,2,3,4,5,6,7,8,9};
    std::set<int>::iterator set_it=iset.begin();
    if(set_it!=iset.end()){
//        *set_it=42;     //错误：set中的关键字是只读
        std::cout << *set_it << std::endl;  //正确：可以读关键字
    }

    //遍历关联容器
    auto map_it1=word_count.cbegin();
    //比较当前迭代器与尾后迭代器
    while (map_it1!=word_count.cend()){
        std::cout << map_it1->first << " occurs "
                  << map_it1->second << " times " << std::endl;
        ++map_it1;
    }

#endif

//添加元素
#ifdef _ADD_ELEMENT_
    //由于map和set包含不重复的关键字，因此插入一个已经存在的元素对容器没有影响
    std::vector<int> ivec={2,4,6,8,2,4,6,8};
    std::set<int> set2;
    set2.insert(ivec.cbegin(),ivec.cend());
    set2.insert(12);
    for(auto &x:set2){
        std::cout << "set2: " << x << std::endl;
    }
    //对于给定的关键字，只有第一个带此关键字的元素才被插入到容器中
    set2.insert({1,3,5,7,1,3,5,7});
    for(auto &x:set2){
        std::cout << "set2: " << x << std::endl;
    }

    //向map添加元素
    std::string word;
    word_count.insert({word,1});
    word_count.insert(std::make_pair(word,1));
    word_count.insert(std::pair<std::string,size_t>(word,1));
    word_count.insert(std::map<std::string,size_t>::value_type(word,1));

    std::vector<int> K;
    for (auto x: K){
        std::cout << 1 << std::endl;
    }

#endif

    //检测insert的返回值(map/set)
    /***
     *insert(或emplace)的返回值依赖于容器类型和参数。对于不重复关键字的容器，
     * 添加单一元素的insert和emplace版本返回一个pair，告诉我们插入操作是否成功
     * pair的first成员是迭代器，指向具有给定关键字的元素
     * second是bool值，指出元素是否插入成功
        若关键字在容器中，则insert什么都不做，返回false
        若关键字不在容器中,元素被插入容器，bool为true
     */
#ifdef _MAP_SET_ADD_POP_
     std::map<std::string,size_t > word_count1;
     std::string word1;
     while(std::cin >> word1 && word1!="q"){
         auto ret=word_count1.insert({word1,1});
         if(!ret.second){
             ++ret.first->second;
         }
         std::cout << "count: " << word_count1[word1];
     }
     for (auto &pair:word_count1){
         std::cout << "first: " << pair.first << std::endl;
         std::cout << "second: " << pair.second << std::endl;
     }
    std::string removal_word="a";
     if (word_count1.erase(removal_word)){
         std::cout << "ok: " << removal_word << " removed\n";
     }else{
         std::cout << "oops: " << removal_word << " not found!\n";
     }

//     std::set<std::string> set_count;
//     std::string word2;
//     while(std::cin >> word2 || word2!="q"){
//         auto ret_set=set_count.insert(word2);
//         if (ret_set.second){
//             std::cout << *ret_set.first << std::endl;
//         }
//     }
#endif

#ifdef _MUTLISET_MUTLIMAP_ADD_
     //向multiset或multimap添加元素
     /***
      * 对于允许重复关键字的容器，接受单个元素的insert操作返回一个指向新元素的迭代器；
      * 这里无须返回一个bool，因为insert总是向这类容器中加入一个新元素
      */
      std::multimap<std::string,std::string> authors;
      authors.insert({"Barth,John","Sot-Weed Factor"});
      auto mul_map=authors.insert({"Barth,John","Lost in the Funhouse"});
      std::string removal_word="Barth,John";
      int num=authors.erase(removal_word);
      if (num){
          std::cout << "ok: " << removal_word << " removed " <<
          "num: " << num << std::endl;
      }else{
          std::cout << "oops: " << removal_word << " not found!\n";
      }


      std::multiset<std::string> set_auth;
      auto mul_set=set_auth.insert("adasa");
#endif

#ifdef _MAP_SUBSCRIPT_OPTIONS_
      std::map<std::string,size_t > word_count2;
      //插入一个关键字为Anna的元素，关联值进行初始化；然后将1赋予它
    std::cout << word_count2["a"] << std::endl;//0
    word_count2["Anna"]=1;

    std::cout << word_count2["Anna"];   //1
    ++word_count2["Anna"];
    std::cout << word_count2["Anna"]; //2
#endif

#ifdef _ELEMENT_VISIT_
    std::set<int> iset1={0,1,2,3,4,5,6,7,8,9};
    //find返回一个迭代器，指向第一个关键字为k的元素，若k不在容器中，则返回尾后迭代器
    iset.find(1);   //返回一个迭代器，指向key==1的元素
    iset.find(11);  //返回一个迭代器，其值等于iset.end()
    //返回关键字等于k的元素的数量。对于不重复关键字的容器，返回值永远是0或1
    iset.count(1);  //返回1
    iset.count(11); //返回0

    int k;
    while (std::cin >> k){
        if(iset.find(k)!=iset.end()){
            std::cout <<  k << " is in iset" << std::endl;
        } else{
            std::cout << k << " is not in set " << std::endl;
        }
    }

    //c.lower_bound(k)返回一个迭代器，指向第一个关键字不小于k的元素
    std::cout << *iset1.lower_bound(4) << std::endl;

    //c.upper_bound(k)返回一个迭代器，指向第一个关键字大于k的元素
    std::cout << *iset1.upper_bound(4) << std::endl;

    //返回一个迭代器pair，表示关键字等于k的元素的范围。若k不存在，pair的两个成员均等于c.end()
    std::cout << *iset1.equal_range(4).first << " " <<
    *iset1.equal_range(4).second<< std::endl;
#endif

#ifdef _ASSOCIATE_CONTAINERS_FIND_ELEMENT_
    //在multimap和multiset中查找元素
    //如果一个multimap或multiset中有多个元素具有给定关键字，则这些元素在容器中会相邻存储
    std::multimap<std::string,std::string> authors;
    std::string author,book;
    while (std::cin >> author && author!="q"){
        std::cin >> book;
        authors.insert({author,book});
    }
    std::string search_item("a"); //要查找的作者
    auto entries=authors.count(search_item);
    auto iter=authors.find(search_item);    //此作者第一本书

    //用一个循环查找此作者的所有著作
    while(entries){
        std::cout << iter->second << std::endl;
        ++iter;
        --entries;
    }

    //beg和end表示对应此作者的元素的范围
    for (auto beg=authors.lower_bound(search_item),end=authors.upper_bound(search_item);
            beg!=end;++beg)   {
        std::cout << beg->second << std::endl;
    }

    //使用equal_range查找关键字匹配范围
    for (auto pos=authors.equal_range(search_item);
        pos.first!=pos.second;++pos.first){
        std::cout << pos.first->second << std::endl;
    }
#endif

    std::ifstream map_file("/home/zxj/桌面/learngit/C++ Prime/example/11-associative_containers/map_file.txt"),
                  transform("/home/zxj/桌面/learngit/C++ Prime/example/11-associative_containers/transform.txt");
    word_transform(map_file,transform);
}

