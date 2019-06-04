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

#ifndef CARTOGRAPHER_COMMON_THREAD_POOL_H_
#define CARTOGRAPHER_COMMON_THREAD_POOL_H_

#include <deque>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "cartographer/common/task.h"

namespace cartographer {
namespace common {

class Task;

class ThreadPoolInterface {
public:
    ThreadPoolInterface() {}
    virtual ~ThreadPoolInterface() {}
    virtual std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task) = 0; // 见下

protected:
    void Execute(Task* task); /// task->Execute()
    void SetThreadPool(Task* task);/// task->SetThreadPool(this)

private:
    friend class Task;

    virtual void NotifyDependenciesCompleted(Task* task) = 0; // 见下
};

// A fixed number of threads working on tasks. Adding a task does not block.
// Tasks may be added whether or not their dependencies are completed.
// When all dependencies of a task are completed, it is queued up for execution
// in a background thread. The queue must be empty before calling the
// destructor. The thread pool will then wait for the currently executing work
// items to finish and then destroy the threads.
class ThreadPool : public ThreadPoolInterface {
public:
    explicit ThreadPool(int num_threads);
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // When the returned weak pointer is expired, 'task' has certainly completed,
    // so dependants no longer need to add it as a dependency.
    std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task)
    LOCKS_EXCLUDED(mutex_) override;

private:
    void DoWork();

    void NotifyDependenciesCompleted(Task* task) LOCKS_EXCLUDED(mutex_) override;

    /**
    * GUARDED_BY(mutex_) -->
    * THREAD_ANNOTATION_ATTRIBUTE__(guarded_by(mutex_)) -->
    * __attribute__((guarded_by(mutex_))
    *  编译器指令，guarded_by属性是为了保证线程安全，使用该属性后，线程要使用相应变量，必须先锁定mutex_
    */
    absl::Mutex mutex_; /// 互斥锁
    bool running_ GUARDED_BY(mutex_) = true;
    std::vector<std::thread> pool_ GUARDED_BY(mutex_); /// 线程池
    std::deque<std::shared_ptr<Task>> task_queue_ GUARDED_BY(mutex_); /// 任务队列
    absl::flat_hash_map<Task*, std::shared_ptr<Task>> tasks_not_ready_ GUARDED_BY(mutex_); /// Task* --> Ptr<Task>的哈希map
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_THREAD_POOL_H_
