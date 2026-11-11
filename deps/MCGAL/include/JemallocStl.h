#ifndef JEMALLOC_ALLOCATOR_H
#define JEMALLOC_ALLOCATOR_H

#include <cstdlib>
// #include <jemalloc/jemalloc.h>
// #include "JemallocAllocator.hpp"
#include <limits>
#include <new>
#include <utility>
#include <queue>
#include <vector>
#include <deque>


// 使用 jemalloc 分配器的 std::vector 定义
// template <typename T> using JemallocVector = std::vector<T, jemallocator::jemallocator<T>>;

// template <typename T> using JemallocDeque = std::deque<T, jemallocator::jemallocator<T>>;

// template <typename T> using JemallocQueue = std::queue<T, jemallocator::jemallocator<T>>;

template <typename T> using JemallocVector = std::vector<T>;

template <typename T> using JemallocDeque = std::deque<T>;

template <typename T> using JemallocQueue = std::queue<T>;

#endif  // JEMALLOC_ALLOCATOR_H
