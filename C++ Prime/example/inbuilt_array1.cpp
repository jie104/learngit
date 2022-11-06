#include <iostream>


int main()
{
//指针运算
    constexpr size_t sz=5;
    int arr[sz]={1,2,3,4,5};
    int *ip=arr;    //等价于 int *ip=&arr[0]
    int *ip2=ip+4;  //ip2指向arr的尾元素 arr[4]

    //正确：arr转化成指向它首元素的指针；p指向arr尾元素的下一个位置
    int *p=arr+sz;  //使用警告：不要解引用
    int *p2=arr+10; //错误：arr只有5个元素，p2的值未定义

    //两个指针相减的结果类型是一种名为ptrdiff_t的标准库类型
    auto n=std::end(arr)-std::begin(arr);


//解引用和指针运算的交互
    int ia[]={0,2,4,6,8};   //含有5个元素的数组
    int last=*(ia+4);

//下标和指针
    //在很多情况下使用数组的名字其实使用的是一个指向数组首元素的指针
    //一个典型的例子是当对数组使用下标运算符时，编译器会自动执行上述转换操作
    int i=ia[2];    //ia转换成指向数组首元素的指针，ia[2]得到(ia+2)所指的元素
    int *p=ia;
    i=*(p+2);

    //只要指针指向的是数组中的元素(或数组中的尾元素的下一个位置)。都可以执行下标运算
    int *p=&ia[2];
    int j=p[1]; //p[1]等价于*(p+1),就是ia[3]表示的那个元素
    int k=p[-2];    //p[-2]是ia[0]表示的那个元素

}