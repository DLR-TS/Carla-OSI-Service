//semaphore from https://stackoverflow.com/questions/4792449/c0x-has-no-semaphores-how-to-synchronize-threads

#include <mutex>
#include <condition_variable>

class Semaphore {
	std::mutex mutex_;
	std::condition_variable condition_;
	unsigned long count_ = 0; // Initialized as locked.

public:
	void release() {
		std::lock_guard<decltype(mutex_)> lock(mutex_);
		++count_;
		condition_.notify_one();
	}

	void acquire() {
		std::unique_lock<decltype(mutex_)> lock(mutex_);
		while (!count_) // Handle spurious wake-ups.
			condition_.wait(lock);
		--count_;
	}

	bool try_acquire() {
		std::lock_guard<decltype(mutex_)> lock(mutex_);
		if (count_) {
			--count_;
			return true;
		}
		return false;
	}
};
