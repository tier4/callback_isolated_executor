#include <sys/syscall.h>

#include "callback_isolated_executor/multi_threaded_executor_internal.hpp"

void MultiThreadedExecutorInternal::pre_spin() {
  if (pre_spinning_.exchange(true)) {
    throw std::runtime_error("pre_spin() called while already pre-spinning");
  }

  {
    std::lock_guard<std::mutex> lock{mtx_};
    assert(threads_.empty() && tids_.empty());
    ready_count_ = 0;
    start_allowed_ = false;
  }

  threads_.reserve(number_of_threads_);
  for (size_t i = 0; i < number_of_threads_; i++) {
    threads_.emplace_back(&MultiThreadedExecutorInternal::run, this);
  }
}

void MultiThreadedExecutorInternal::run() {
  auto tid = static_cast<pid_t>(syscall(SYS_gettid));
  bool will_notify = false;

  {
    std::lock_guard<std::mutex> lock{wait_mutex_};
    tids_.push_back(tid);
    ready_count_++;
    if (ready_count_ == number_of_threads_) {
      will_notify = true;
    }
  }

  if (will_notify) {
    cv_all_ready_.notify_one();
  }

  {
    std::unique_lock<std::mutex> lock{mtx_};
    cv_start_.wait(lock, [this]() { return start_allowed_; });
  }

  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_exec;

    {
      std::lock_guard wait_lock{wait_mutex_};

      if (!rclcpp::ok(this->context_) || !spinning.load()) {
        return;
      }

      if (!get_next_executable(any_exec, next_exec_timeout_)) {
        continue;
      }
    }

    if (yield_before_execute_) {
      std::this_thread::yield();
    }

    execute_any_executable(any_exec);

    // Clear the callback_group to prevent the AnyExecutable destructor from
    // resetting the callback group `can_be_taken_from`
    any_exec.callback_group.reset();
  }
}

void MultiThreadedExecutorInternal::spin() {
  if (!pre_spinning_.load()) {
    throw std::runtime_error("spin() called without pre_spin()");
  }

  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }

  RCPPUTILS_SCOPE_EXIT(pre_spinning_.store(false););
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false););

  {
    std::unique_lock<std::mutex> lock{mtx_};
    cv_all_ready_.wait(lock,
                       [this]() { return ready_count_ == number_of_threads_; });
  }

  {
    std::lock_guard<std::mutex> lock{mtx_};
    start_allowed_ = true;
  }

  cv_start_.notify_all();

  for (auto &thread : threads_) {
    thread.join();
  }
}

std::vector<pid_t> MultiThreadedExecutorInternal::get_thread_ids() {
  if (!pre_spinning_.load()) {
    throw std::runtime_error("get_thread_ids() called without pre_spin()");
  }

  {
    std::unique_lock<std::mutex> lock{mtx_};
    cv_all_ready_.wait(lock,
                       [this]() { return ready_count_ == number_of_threads_; });
  }

  std::lock_guard<std::mutex> lock{mtx_};
  return tids_;
}
