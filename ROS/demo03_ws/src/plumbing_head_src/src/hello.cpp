#include "ros/ros.h"
#include "plumbing_head_src/hello.h"

    namespace hello_ns {

    void MyHello::run(){
        ROS_INFO("源文件中的run函数");
    }
    }

int main(int argc,char* argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"hello_head");
    //函数调用
    hello_ns::MyHello myhello;
    myhello.run();

    return 0;
}