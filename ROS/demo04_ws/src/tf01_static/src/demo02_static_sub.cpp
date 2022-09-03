#include "ros/ros.h"
#include "tf2_ros/transform_listener.h" //用来创建订阅对象
#include "tf2_ros/buffer.h" //订阅的数据缓存到buffer中
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //坐标转换过程需要用到

/*
    订阅方：订阅发布的坐标系相对关系，传入一个坐标点，调用tf实现转换

    流程：
        1.包含头文件
        2.编码、初始化、NodeHandle()
        3.创建订阅对象： ---> 订阅坐标相对关系
        4.组织一个坐标点数据；
        5.转换算法、需要调用TF内置实现
        6.最后输出。
*/

int main(int argc,char* argv[])
{
        // 2.编码、初始化、NodeHandle()
        setlocale(LC_ALL,"");
        ros::init(argc,argv,"static_sub");
        ros::NodeHandle nh;
        // 3.创建订阅对象： ---> 订阅坐标相对关系
        //3-1.创建一个buffer缓存
        tf2_ros::Buffer buffer;
        //3-2.再创建监听对象（监听对象可以将订阅的数据存入buffer)
        tf2_ros::TransformListener listener(buffer);
        // 4.组织一个坐标点数据；
        geometry_msgs::PointStamped ps;
        ps.header.frame_id="laser";
        ps.header.stamp=ros::Time::now();
        ps.point.x=2.0;
        ps.point.y=3.0;
        ps.point.z=5.0;

        //添加休眠
        // ros::Duration(2).sleep();   //休眠2s
        // 5.转换算法、需要调用TF内置实现
        ros::Rate rate(10);
        while (ros::ok())
        {
            //核心代码实现 ----将ps转换成相对于 base_link坐标点
            geometry_msgs::PointStamped ps_out;
            /*
                调用了buffer的转换函数 transform
                参数1：被转换的坐标点
                参数2：目标坐标系
                返回值：输出坐标点

                PS：调用时必须包含头文件 tf2_geometry_msgs/tf2_geometry_msgs.h
                PS:运行时存在的问题：抛出一个异常，base_link不存在
                    原因：订阅数据是耗时操作，可能在调用transform转换函数时，坐标系的
                        相对关系还没订阅，因此出现异常
                    解决：
                        方案1：在调用转换函数前，执行休眠
                        方案2：进行异常处理(建议)  try....catch ...
            
            */
                //将坐标系laser下的ps转换为坐标系base_link下的坐标
            try{
                ps_out=buffer.transform(ps,"base_link");

                // 6.最后输出。
                ROS_INFO("转换后的坐标值：(%.2f,%.2f,%.2f),参考的坐标系：%s",
                            ps_out.point.x,
                            ps_out.point.y,
                            ps_out.point.z,
                            ps_out.header.frame_id.c_str());
            }catch(const std::exception& e){
                ROS_INFO("异常信息：%s",e.what());
            }

            rate.sleep();
            // ros::spinOnce();
        }


    return 0;
}