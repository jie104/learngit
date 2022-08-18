/**
 * @file thread_pool.h
 * @author zmy (626670628@qq.com)
 * @brief 线程池
 * @version 0.1
 * @date 2022-04-12
 *
 *
 */

#ifndef BAG_THREAD_POOL_THREAD_POOL_H
#define BAG_THREAD_POOL_THREAD_POOL_H

#include "containers/threadsafe_queue.h"

#include <atomic>
#include <functional>
#include <future>
#include <memory>
#include <thread>

namespace concurrency
{
    namespace pool
    {
        class ThreadPool
        {
            using Task = std::function<void()>;

        public:
            ThreadPool(const int thread_count = std::thread::hardware_concurrency());
            ~ThreadPool();
            void stop();

            template <typename Func, typename... Arg>
            void submit(Func func, Arg... args)
            {
                global_queue_->push([task = std::move(func), args...]()
                                    { task(args...); });
                return;
            }

        private:
            void threadWorker(const unsigned thread_id, std::shared_future<void> ready);
            bool popFromQueues(Task &task);

        private:
            int thread_count_;
            std::atomic<bool> done_;
            static thread_local unsigned thread_id_;
            std::unique_ptr<containers::TSQueue<Task>> global_queue_;
            std::vector<std::thread> threads_;
        };

    } // namespace pool

} // namespace thread_pool

#endif