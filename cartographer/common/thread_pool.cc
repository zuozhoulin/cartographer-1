/*
* Copyright 2016 The Cartographer Authors
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include "cartographer/common/thread_pool.h"

#ifndef WIN32
#include <unistd.h>
#endif
#include <algorithm>
#include <chrono>
#include <numeric>

#include "absl/memory/memory.h"
#include "cartographer/common/task.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {

void ThreadPoolInterface::Execute(Task* task) { task->Execute(); }

void ThreadPoolInterface::SetThreadPool(Task* task) {
    task->SetThreadPool(this);
}

ThreadPool::ThreadPool(int num_threads) {
    absl::MutexLock locker(&mutex_);
    for (int i = 0; i != num_threads; ++i) {
        pool_.emplace_back([this]() { ThreadPool::DoWork(); });
        /// 添加线程 --- 线程执行的函数为lambda
    }
}

ThreadPool::~ThreadPool() {
    {
        absl::MutexLock locker(&mutex_);
        CHECK(running_);
        running_ = false; // running设置为false，使DoWork()函数退出循环
    }
    for (std::thread& thread : pool_) {
        thread.join();/// 等待所有线程结束，保证安全析构线程池资源
    }
}

void ThreadPool::NotifyDependenciesCompleted(Task* task) {
    absl::MutexLock locker(&mutex_);
    auto it = tasks_not_ready_.find(task);
    CHECK(it != tasks_not_ready_.end());
    task_queue_.push_back(it->second);
    tasks_not_ready_.erase(it);
}

std::weak_ptr<Task> ThreadPool::Schedule(std::unique_ptr<Task> task) {
    std::shared_ptr<Task> shared_task;
    {
        absl::MutexLock locker(&mutex_);
        auto insert_result =
                tasks_not_ready_.insert(std::make_pair(task.get(), std::move(task)));
        CHECK(insert_result.second) << "Schedule called twice";
        shared_task = insert_result.first->second;
    }
    SetThreadPool(shared_task.get());
    return shared_task;
}

void ThreadPool::DoWork() {
#ifdef __linux__
    // This changes the per-thread nice level of the current thread on Linux. We
    // do this so that the background work done by the thread pool is not taking
    // away CPU resources from more important foreground threads.
    CHECK_NE(nice(10), -1);
    /// nice函数改变当前进程优先级，值越高，优先级越低，设置失败则返回 -1
    /// 目的是不希望这里的线程过多竞争CPU资源
#endif
    const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
        return !task_queue_.empty() || !running_; /// 任务队列非空或者running标记为false时返回true。
    };
    for (;;) {
        std::shared_ptr<Task> task;
        {
            absl::MutexLock locker(&mutex_);
            mutex_.Await(absl::Condition(&predicate)); /// 阻塞，等待条件为真

            /
            if (!task_queue_.empty()) { // 任务队列非空，则取出需要执行的任务
                task = std::move(task_queue_.front());
                task_queue_.pop_front();
            } else if (!running_) {
                return; /// running标记为false时，退出循环，DoWork函数（线程）结束。
            }
        }
        CHECK(task);
        CHECK_EQ(task->GetState(), common::Task::DEPENDENCIES_COMPLETED);
        Execute(task.get());
        /// 执行任务 task->Excute();
    }
}

}  // namespace common
}  // namespace cartographer
