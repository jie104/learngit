/**
 * @file thread_pool.cpp
 * @author zmy (626670628@qq.com)
 * @brief 线程池实现
 * @version 0.1
 * @date 2022-04-12
 *
 *
 */

#include "thread_pool/bag_thread_pool.h"

namespace concurrency
{
    namespace pool
    {
        thread_local unsigned ThreadPool::thread_id_(std::numeric_limits<unsigned>::max());
        
        ThreadPool::ThreadPool(const int thread_count)
            : thread_count_(thread_count < std::thread::hardware_concurrency() ? thread_count : std::thread::hardware_concurrency()),
              done_(false), global_queue_(std::make_unique<containers::TSQueue<Task>>())
        {
            threads_.reserve(thread_count_);
            auto is_ready = std::promise<void>();
            std::shared_future<void> ready = is_ready.get_future().share();

            for (size_t i = 0; i < thread_count_; ++i)
            {
                threads_.emplace_back(std::thread(&ThreadPool::threadWorker, this, i, ready));
            }
            is_ready.set_value();
        }

        ThreadPool::~ThreadPool()
        {
            done_.store(true);
            for (auto &th : threads_)
            {
                th.join();
            }
        }

        void ThreadPool::threadWorker(const unsigned thread_id, std::shared_future<void> ready)
        {
            thread_id_ = thread_id;
            ready.wait();
            while (!done_)
            {
                Task task;
                if (popFromQueues(task))
                {
                    task();
                }
            }
        }

        bool ThreadPool::popFromQueues(Task &task)
        {
            return global_queue_->wait_and_pop(task);
        }

        void ThreadPool::stop()
        {
            done_.store(true);
            global_queue_->clear();
        }
    }
}
