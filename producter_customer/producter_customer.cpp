//
// Created by zxj on 2022/11/15.
//

#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <unistd.h> //ubuntu系统下
#include<pthread.h>

//生产者数量
#define PRODUCT_SIZE 20
//消费者数量
#define CUSTOMER_SIZE 1
//最大产品数量
#define MAX_SIZE 10
//互斥锁
std::mutex mut;
//条件变量
std::condition_variable con;
//队列，模拟缓冲区
std::queue<int> que;

void Producter()
{
    while(true){
        sleep(10);
        //获取全局的互斥锁mut，并且自动进行加锁操作。值得注意的是，lck作为局部变量，
        //当一次循环结束后，会自动调用自己的析构函数，来对锁进行解锁操作
        std::unique_lock<std::mutex> lck(mut);
        //wait操作可以返回，程序并不是一直在wait处，因此，如果将条件判断改为if，那么判断一次之后
        //程序将会继续往下执行，会导致与预期目标不符的结果
        while (que.size()>MAX_SIZE){
            //条件变量的成员函数首先会对锁进行解锁操作，然后会将线程挂起
            con.wait(lck);
        }
        int data=std::rand();
        que.push(data);
        std::cout << std::this_thread::get_id() << "生产了产品" << data << std::endl;
        //当条件变量被notify_*系列函数唤醒时，会将锁再次进行加锁
        con.notify_all();
    }
}

void Customer()
{
    while (true)
    {
        std::unique_lock <std::mutex> lck(mut);
        while (que.empty())
        {
            con.wait(lck);
        }
        std::cout << std::this_thread::get_id() << "消费了产品：" << que.front() << std::endl;
        que.pop();
        con.notify_all();
    }
}
int main()
{
//    std::vector<std::thread> threadPoll;
    std::thread productor(Producter);
    std::thread consumer(Customer);
    //创建生产者和消费者
//    for (int i = 0; i < PRODUCT_SIZE; ++i)
//    {
//        std::cout << "i: " << i << std::endl;
//        threadPoll.push_back(std::thread(Producter));
//    }
//    for (int i = 0; i < CUSTOMER_SIZE; ++i)
//    {
//        threadPoll.push_back(std::thread(Customer));
//    }
//
//
//    for (int i = 0; i < PRODUCT_SIZE + CUSTOMER_SIZE; ++i)
//    {
//        //等待子线程执行完成之后，主线程才继续执行，此时主线程会释放掉执行完成后的子线程的资源。
//        //谁调用了join()函数？threadPoll[i]对象调用了join()函数，因此必须等待threadPoll[i]线程执行结束，threadPoll[i].join()才能得到返回。
//        //threadPoll[i]在哪个线程环境下调用了join()函数？threadPoll[i]是在主线程的环境下调用了join()函数，因此主线程要等待threadPoll[i]的线程工作做完，否则主线程将一直处于block状态；
//        // 这里不要搞混的是threadPoll[i]真正做的任务是在另一个线程做的，但是threadPoll[i]调用join()函数的动作是在主线程环境下做的。
//        threadPoll[i].join();
//    }
    productor.join();
    consumer.join();
    return 0;
}
