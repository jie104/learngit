//
// Created by zxj on 2023/4/13.
//
#include <string>
#include <memory>


class StrVec{
public:
    StrVec():   //allocator成员默认初始化
        elements(nullptr),first_free(nullptr),cap(nullptr){}

    //移动构造函数
    //与拷贝构造函数不同，移动构造函数不分配任何新内存
    //不抛出异常的移动构造函数和移动赋值运算符必须标记未noexcept
    StrVec(StrVec &&s) noexcept //移动操作不抛出任何异常
    //成员初始化器接管ss中的资源
    :elements(s.elements),first_free(s.first_free),cap(s.cap)
    {
        //令s进入这样的状态---对其运行析构函数是安全的
        //如果忘记改变s.first_free，s.elements，s.cap
        // 则销毁后源对象会释放掉我们刚刚移动的内存
        s.elements=s.first_free=s.cap= nullptr;
    }


    StrVec(const StrVec&);
    StrVec &operator=(const StrVec&);
    ~StrVec();
    void push_back(const std::string&);
    size_t size() const {return first_free-elements;}
    size_t capacity() const {return cap-elements;}
    std::string *begin() const {return elements;}
    std::string *end() const {return first_free;}
private:
    static std::allocator<std::string > alloc;
    //被添加元素的函数调用
    void chk_n_alloc(){if (size()==capacity()) reallocate();};

    //工具函数，被拷贝构造函数、赋值运算符和析构函数所使用
    std::pair<std::string*,std::string*> alloc_n_copy(const std::string*,const std::string*);
    void free();        //释放内存
    void reallocate();  //获得更多内存并靠背1已有元素
    std::string *elements;  //指向数组首元素的指针
    std::string *first_free;    //指向数组第一个空闲元素的指针
    std::string *cap;   //指向数组尾后位置的指针
};

void StrVec::push_back(const std::string &s) {
    chk_n_alloc();  //确保有空间容纳新元素
    //在first_free指向的元素中构造s的副本
    alloc.construct(first_free++,s);
}

std::pair<std::string*,std::string*>
StrVec::alloc_n_copy(const std::string *b, const std::string *e) {
    //分配空间保存给定范围中的元素
    auto data=alloc.allocate(e-b);
    //初始化并返回一个pair,该pair由data和uninitialized_copy的返回值构成
    //返回值second成员是uninitialized_copy的返回值，此值是一个指针，指向最后一个构造元素之后的位置
    return {data,std::uninitialized_copy(b,e,data)};
}

void StrVec::free() {
    //不能传递給deallocate一个空指针，如果elements为0，函数什么都不做
    if(elements){
        for(auto p=first_free;p!=elements;){
            alloc.destroy(--p); //调用string的析构函数释放string自己分配的内存空间
        }
        //一旦元素被销毁，就调用deallocate来释放本StrVec对象分配的内存空间
        alloc.deallocate(elements,cap-elements);
    }
}

//拷贝控制成员
StrVec::StrVec(const StrVec &s) {
    //调用alloc_n_copy分配空间以容纳与s中一样多的元素
    auto newdata= alloc_n_copy(s.begin(),s.end());
    elements=newdata.first;
    first_free=cap=newdata.second;
}

StrVec::~StrVec() {free();}

StrVec &StrVec::operator=(const StrVec &rhs) {
    auto data= alloc_n_copy(rhs.begin(),rhs.end());
    free();
    elements=data.first;
    first_free=cap=data.second;
    return *this;
}

void StrVec::reallocate() {
    //分配当前大小两倍的内存空间
    auto newcapacity=size()? 2*size():1;
    //分配新内存
    auto newdata=alloc.allocate(newcapacity);
    //将数据从旧内存移到新内存
    auto dest=newdata;
    auto elem=elements;
    for(size_t i=0;i!=size();++i)
        alloc.construct(dest++,std::move(*elem++));
    free(); //一旦移动完元素就释放旧内存空间
    //更新数据结构执行新元素
    elements=newdata;
    first_free=dest;
    cap=elements+newcapacity;
}