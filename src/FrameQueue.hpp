#include <queue>
#include <vector>

template <typename T> class FrameQueue {
 public:
    FrameQueue(size_t capacity) : capacity_(capacity) {
    }

    void push(const T &value) {
        if (frame_queue_.size() == capacity_) {
            frame_queue_.pop();
        }
        frame_queue_.push(value);
    }

    T front() {
        return frame_queue_.front();
    }

    T back() {
        return frame_queue_.back();
    }

    void pop() {
        frame_queue_.pop();
    }

    size_t size() {
        return frame_queue_.size();
    }

 private:
    std::queue<T> frame_queue_;
    size_t capacity_;
};
