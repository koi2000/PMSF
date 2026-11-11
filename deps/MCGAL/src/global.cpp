
#include "Global.h"

namespace MCGAL {
ContextPool& contextPool = ContextPool::getInstance();
// const int num_threads = std::thread::hardware_concurrency();
ThreadPool& threadPool = ThreadPool::getInstance();

}