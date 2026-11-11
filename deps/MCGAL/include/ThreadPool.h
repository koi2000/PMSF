#ifndef THREAD_POOL
#define THREAD_POOL
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <queue>
#include <shared_mutex>
#include <string>
#include <thread>
#include <time.h>
#include <utility>
#include <vector>
namespace MCGAL {

#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <queue>
#include <shared_mutex>
#include <string>
#include <thread>
#include <time.h>
#include <utility>
#include <vector>

template <typename T> struct safe_queue {
    std::queue<T> que;
    std::shared_mutex _m;
    bool empty() {
        std::shared_lock<std::shared_mutex> lc(_m);
        return que.empty();
    }
    auto size() {
        std::shared_lock<std::shared_mutex> lc(_m);
        return que.size();
    }
    void push(T& t) {
        std::unique_lock<std::shared_mutex> lc(_m);
        que.push(t);
    }
    bool pop(T& t) {
        std::unique_lock<std::shared_mutex> lc(_m);
        if (que.empty())
            return false;
        t = std::move(que.front());
        que.pop();
        return true;
    }
};
class ThreadPool {
  private:
    class worker {
      public:
        ThreadPool* pool;
        worker(ThreadPool* _pool) : pool{_pool} {}

        void operator()() {
            while (!pool->is_shut_down) {
                {
                    std::unique_lock<std::mutex> lock(pool->_m);
                    pool->cv.wait(lock, [this]() { return this->pool->is_shut_down || !this->pool->que.empty(); });
                }
                std::function<void()> func;
                bool flag = pool->que.pop(func);
                if (flag) {
                    func();
                }
            }
        }
    };

  public:
    bool is_shut_down;
    safe_queue<std::function<void()>> que;
    std::vector<std::thread> threads;
    int threadCount_ = 0;
    std::once_flag startOnce_;
    bool started_ = false;
    std::mutex _m;
    std::condition_variable cv;
    // perf（2026-11-08 perf record 实测）：原实现在**静态初始化**阶段就按
    // hardware_concurrency() 创建线程（本机 128 个），即使整个进程一次 submit 都没有：
    //   启动 exec→main ≈ 20–25 ms（线程栈 mmap/TLS），退出 join+munmap ≈ 25 ms。
    // PPMC（以及当前全部生产路径）都不使用该池，因此改为**首次 submit 时惰性创建**；
    // 语义不变（真正使用线程池的调用者仍拿到同样数量的 worker）。
    ThreadPool(int n) : is_shut_down{false}, threadCount_{n} {}
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool(ThreadPool&&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    ThreadPool& operator=(ThreadPool&&) = delete;

    static ThreadPool& getInstance() {
        static ThreadPool instance(std::thread::hardware_concurrency());
        return instance;
    }

    template <typename F, typename... Args> auto submit(F&& f, Args&&... args) -> std::future<decltype(f(args...))> {
        // ── D22 修复 ②：**按值捕获**可调用对象，不要按引用捕获形参 f ────────────
        // 原来写的是 `[&f, args...]`。`f` 是 `F&&` 形参，会绑定到**右值** ——
        // `pool.submit([]{...})` 正是最自然的写法，而该临时对象在本语句结束时
        // 就析构了，任务却是**稍后在工作线程上**执行的 ⇒ 调用悬垂可调用对象 = UB。
        // 实测（-O2, g++13）：带捕获的临时 λ 必崩，栈落在
        //   `submit<main::{lambda()#1}>(lambda&&)::{lambda()#1}::_M_invoke`；
        // 连**寿命不及析构**的局部 λ 也一样崩（main 返回后静态析构才执行任务）。
        // 本文件自己的 `~ThreadPool` 就踩这条：`submit([](){})` 传的也是右值，
        // 只因**无捕获 λ 的 operator() 不解引用 this** 才一直侥幸能跑。
        // 修法：按值/移动捕获，让可调用对象的寿命覆盖任务的执行期。
        ensureStarted();
        std::function<decltype(f(args...))()> func
            = [fn = std::forward<F>(f), args...]() mutable { return fn(args...); };
        auto task_ptr = std::make_shared<std::packaged_task<decltype(f(args...))()>>(func);
        std::function<void()> warpper_func = [task_ptr]() { (*task_ptr)(); };
        que.push(warpper_func);
        cv.notify_one();
        return task_ptr->get_future();
    }
    
    void ensureStarted() {
        std::call_once(startOnce_, [this] {
            threads.reserve(static_cast<std::size_t>(threadCount_));
            for (int i = 0; i < threadCount_; ++i)
                threads.emplace_back(worker(this));
            started_ = true;
        });
    }

    ~ThreadPool() {
        if (!started_) {
            return;  // 惰性池从未启动过：没有任何 worker 需要 drain/join
        }
        auto f = submit([]() {});
        f.get();
        // ── D22 修复：关停标志必须**在 _m 保护下**写 ──────────────────────
        // worker 在 `cv.wait(lock, pred)` 里**持 _m** 读 `is_shut_down`；
        // 这里若无锁写，就会出现**丢唤醒**：
        //   ① worker 持 _m，判定 pred()==false；
        //   ② 本析构函数（不持 _m）写 true 并 notify_all()；
        //   ③ worker 才进入 cv.wait(lock) 原子解锁并阻塞 —— 而通知**已经发过了**
        //   ⇒ worker 永远睡下去 ⇒ 下面 join() 永久阻塞 ⇒ 进程挂在退出处。
        // 现象与 D22 现场一致：worker 全停在 pool 的 condvar 上、
        // 另有线程在等某个 tid；且因是竞态窗口而**间歇**、因线程池是**共享单例**
        // 而**跨算法**（与 D21 同路径的合理来源）。
        // 修法：把写放在与 worker 判定**同一把锁**下 ⇒ 判定与告知严格有序。
        {
            std::lock_guard<std::mutex> lk(_m);
            is_shut_down = true;
        }
        cv.notify_all();  // 通知，唤醒所有工作线程
        for (auto& t : threads) {
            if (t.joinable())
                t.join();
        }
    }
};
}  // namespace MCGAL
#endif