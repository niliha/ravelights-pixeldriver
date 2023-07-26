#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>

template <typename T> class BlockingRingBuffer {

 public:
    BlockingRingBuffer(size_t capacity) : capacity_(capacity) {
    }

    void push(T &&item) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (buffer_.size() == capacity_) {
                buffer_.pop_front();
            }
            buffer_.push_back(std::move(item));
        }
        isNotEmpty_.notify_one();
    }

    void pop(T &item) {
        std::unique_lock<std::mutex> lock(mutex_);
        isNotEmpty_.wait(lock, [this]() { return !buffer_.empty(); });
        item = std::move(buffer_.front());
        buffer_.pop_front();
    }

 private:
    std::deque<T> buffer_;
    size_t capacity_;
    std::mutex mutex_;
    std::condition_variable isNotEmpty_;
};
