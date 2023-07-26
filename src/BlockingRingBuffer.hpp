#include <condition_variable>
#include <mutex>
#include <deque>

template <typename T> class BlockingRingBuffer {

 public:
    BlockingRingBuffer(size_t capacity) : capacity(capacity) {
    }

    void push(T &&item) {
        {
            std::unique_lock<std::mutex> lk(mutex);
            if (content.size() == capacity) {
                content.pop_front();
            }
            content.push_back(std::move(item));
        }
        not_empty.notify_one();
    }

    void pop(T &item) {
        std::unique_lock<std::mutex> lk(mutex);
        not_empty.wait(lk, [this]() { return !content.empty(); });
        item = std::move(content.front());
        content.pop_front();
    }

 private:
    std::deque<T> content;
    size_t capacity;
    std::mutex mutex;
    std::condition_variable not_empty;
};
