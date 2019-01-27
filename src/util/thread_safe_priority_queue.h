// Copyright 2017 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================
#ifndef SRC_UTIL_THREAD_SAFE_PRIORITY_QUEUE_H_
#define SRC_UTIL_THREAD_SAFE_PRIORITY_QUEUE_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <vector>

namespace threadsafe {

template <class DataType, class PriorityType>
class ThreadSafePriorityQueue {
 public:
  struct PriorityQueueElement {
    DataType data;
    PriorityType priority;
  };

  struct Compare {
    bool operator()(const PriorityQueueElement& first,
                    const PriorityQueueElement& second) {
      return first.priority > second.priority;
    }
  };

 private:
  std::priority_queue<PriorityQueueElement,
                      std::vector<PriorityQueueElement>, Compare> queue_;

  std::condition_variable read_cv_;
  std::condition_variable write_cv_;
  std::mutex access_mutex_;
  std::atomic_bool has_data_;

  const unsigned int kCriticalValue = 10;
  const float kSmallTime = 20;  // ms
  const float kLongTime = 25;  // ms

  std::atomic_bool is_running_;

 public:
  ThreadSafePriorityQueue() : has_data_(false), is_running_(true) {}
  ~ThreadSafePriorityQueue() {}

  // Ensures that no reads or writes will be blocking.
  void Shutdown() {
    has_data_ = true;
    is_running_ = false;
    read_cv_.notify_all();
  }

  // Blocks waiting to add to the queue. Will yield to reads.
  void Add(DataType data, PriorityType priority) {
    if (is_running_) {
      // Transfer from the output actor to the local actor.
      std::unique_lock<std::mutex> guard(access_mutex_);
      if (queue_.size() > kCriticalValue) {
        read_cv_.wait_for(guard,
                          std::chrono::duration<float, std::milli>(kLongTime));
      }
      PriorityQueueElement element;
      element.data = data;
      element.priority = priority;
      queue_.push(element);
      has_data_ = true;
    }  // Lock loses scope here.
    write_cv_.notify_all();
  }

  // Blocks waiting to read some data from the queue and empty all data from it.
  void ReadAllAndEmpty(std::vector<DataType>* output_queue) {
    if (has_data_ && is_running_) {
      std::unique_lock<std::mutex> guard(access_mutex_);

      while (!queue_.empty()) {
        output_queue->push_back(queue_.top().data);
        queue_.pop();
      }

      has_data_ = false;
      read_cv_.notify_all();
    }
  }

  // Blocks waiting to read some data from the queue and empty all data from it.
  void BlockingReadAllAndEmpty(std::vector<DataType>* output_queue) {
    {
      std::unique_lock<std::mutex> guard(access_mutex_);
      while (is_running_ && !has_data_) {
        write_cv_.wait_for(guard,
                          std::chrono::duration<float, std::milli>(kSmallTime));
      }

      while (!queue_.empty()) {
        output_queue->push_back(queue_.top().data);
        queue_.pop();
      }

      has_data_ = false;
      read_cv_.notify_all();
    }
  }
};

}  //  namespace threadsafe
#endif  // SRC_UTIL_THREAD_SAFE_PRIORITY_QUEUE_H_
