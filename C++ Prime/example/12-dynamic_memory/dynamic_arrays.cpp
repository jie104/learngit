//
// Created by zxj on 2023/2/21.
//
#include <iostream>
#include <new>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>

int get_size(){
    return 4;
}

int main()
{
#ifdef _NEW_AND_ARRAYS_
///new和数组
    /***
     * 让new分配一个对象，需要在类型名后跟一对方括号，其中指明要分配的对象数目
     */
     //方括号中的大小必须是整型，但不必是常量
     int *pia=new int[get_size()];  //pia指向第一个int
     std::cout << "pia[0]: " << *pia << std::endl;

     typedef int arrT[42];  //arrT表示42个int的数组类型
     int *p=new arrT;   //分配一个42个int的数组；p指向第一个int

 ///初始化动态分配对象的数组
    //默认情况下，new分配的对象，不管是单个分配还是数组中的，都是默认初始化的
    int *pia1=new int[10];  //10个未初始化的int
    int *pia2=new int[10]();    //10个初始化为0的int
    std::string *psa=new std::string[10];   //10个空string
    std::string *psa2=new std::string[10]();    //10个空string

    //新标准中，可以提供一个元素初始化器的花括号列表
    int *pia3=new int[10]{0,1,2,3,4,5,6,7,8,9};
    //10个string,前四个用給定初始化器初始化，剩余的进行值初始化
    std::string *psa3=new std::string[10]{"1","an","the",std::string(3,'x')};

///动态分配一个空数组是合法的
    size_t n=get_size();
    int *p9=new int[n];
    for(int *q=p;q!=p+n;++q){}

    //对于0元素的数组，此指针保证与new返回的其他指针都不同，对于零长度的数组来说，
    //此指针就像尾后指针一样；此指针不能解引用，它不指向任何元素
    char arr[0];    //错误：不能定义长度为0的数组
    char *cp=new char[0];   //正确，但cp不能解引用




//    double a=0.68,b=1.19,c=1.352;
//    double cos_theta=(a*a+b*b-c*c)/(2*a*b);
//    std::cout << "cos_theta: " << cos_theta << ",theta: " << std::acos(cos_theta) << std::endl;

//    Eigen::Vector2d A(1.7534,-0.53109),B(2.5723,0.57262),C(1.882,0.64434);
//    double AB=(A-B).norm();
//    double AC=(A-C).norm();
//    double BC=(B-C).norm();
//    double cos_theta=(AC*AC+BC*BC-AB*AB)/(2*BC*AC);
//    std::cout << "cos_theta: " << cos_theta << ",theta: " << std::acos(cos_theta)*180/M_PI << std::endl;

///释放动态数组
    //为释放动态数组，使用特殊形式的delete————在指针前面加上一个空括号对
    delete p;   //p必须指向一个动态分配的对象或为空
    delete [] p9;   //pa必须指向一个动态分配的数组或为空

    //使用类型别名定义一个数组类型时，在new表达式中不使用[],但在释放p时，必须使用[]
    typedef int arrT[42];
    int *p10=new arrT;
    delete [] p10;

///智能指针和动态数组
    //unique_ptr管理动态数组，必须在对象类型后面跟一对空方括号
    std::unique_ptr<int [] > up(new int[10]);   //up指向一个包含10个未初始化的数组
    up.release();   //自动用delete[]销毁其指针

    //当一个unique_ptr指向的一个数组时，不能使用点和箭头成员运算符
    //当一个unique_ptr指向一个数组时，可以使用下标运算符来访问数组中元素
    for(size_t i=0;i!=10;++i){
        up[i]=i;
    }

    //为使用shared_ptr,必须提供一个删除器
    std::shared_ptr<int> sp(new int[10],[](int* p){delete[] p;});
    sp.reset(); //使用lambda释放数组

    //shared_ptr不支持动态数组管理这一特性会影响我们如何访问数组中元素
    for(size_t i=0;i!=10;++i){
        *(sp.get()+i)=i;    //使用get获取内置指针
    }
#endif

///allocator类
/***
 * 可以将内存分配和对象构造分类，只有在真正需要的时候，才能真正执行对象创建操作
 */

//    int n=10;
//    std::string *const p=new std::string[n];
//    std::string s;
//    std::string *q=p;
////    std::string * p1=new std::string[n];
////    std::string *const q1=p1;
//    while (std::cin >> s && q!=p+n)
//        *q++=s;
//    const size_t size=q-p;
//    delete[] p;

    //allocate提供一种类型感知的内存分配方法，它分配的内存是原始的、未构造的
    int n=10;
    std::allocator<std::string> alloc;  //可以分配string的allocator对象
    auto p10=alloc.allocate(n); //分配n个未初始化的string

    //allocate分配·未构造的内存
    /***
     * construct成员函数接受一个指针和0个或多个额外参数，在给定位置构造一个元素
     * 额外参数用来初始化构造对象，类似make_shared
     */
     auto q1=p10;
     alloc.construct(q1++);  //*q为空字符串
     alloc.construct(q1++,10,'c');   //*q为ccccccccccc
     alloc.construct(q1++,"hi"); //*q为hi!

     auto p11=p10;
    for(int i=0;i<n;i++){
        std::cout << "p10: " << *p11++ << std::endl;
    }

    //还未构造对象的情况下，使用原始内存是错误的
//    std::cout << "..." << *++p10 << std::endl; //正确：s使用string的输出运算符
    std::cout << "wqqe: " << *q1 << std::endl;  //错误：q指向未构造的内存

    //用完对象后，必须对每个构造的元素调用destory来销毁他，函数destory接受一个指针，
    //对指向的对象执行构造函数
    while(q1!=p10){
        alloc.destroy(--q1);
    }

    //一旦元素被摧毁后，可以重新使用这部分内存保存其他string，也可将其归还系统。
    //释放内存通过调用deallocate来完成
    /***
     * 传递给deallocate的指针不能为空，必须指向由allocate分配的内存
     * 且传递给deallocate的大小参数必须与调用allocated分配内存时提供的大小参数一样
     */
    alloc.deallocate(p10,n);

///拷贝和填充未初始化内存的算法
    //分配比vi中元素占用空间大一倍的动态内存
    std::allocator<int> alloc1;
    std::vector<int> vi={1,2,3,4,5,6,7,8};
   auto p12=alloc1.allocate(vi.size()*2);
   //通过拷贝vi中的元素来构造从p开始的元素
   auto q=std::uninitialized_copy(vi.begin(),vi.end(),p12);
   //将剩余元素初始化为42
   std::uninitialized_fill_n(q,vi.size(),42);
   int k=0;
   while (k <2*vi.size()){
       std::cout << *(p12++)  <<std::endl;
       k++;
   }
}

