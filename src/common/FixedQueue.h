#pragma once
#include <stdexcept>
#include <vector>

template <typename T> class FixedQueue {
    std::vector<T> buf;
    size_t head = 0;
    size_t size = 0;
    const size_t capacity;

  public:
    explicit FixedQueue(size_t cap) : buf(cap), capacity(cap) {}

    void push(const T& item) {
        if (size == capacity)
            throw std::overflow_error("Queue full");
        buf[(head + size) % capacity] = item;
        ++size;
    }

    void pop() {
        if (size == 0)
            throw std::underflow_error("Queue empty");
        head = (head + 1) % capacity;
        --size;
    }

    T& front() {
        if (size == 0)
            throw std::underflow_error("Queue empty");
        return buf[head];
    }

    bool empty() const {
        return size == 0;
    }
    bool full() const {
        return size == capacity;
    }
};