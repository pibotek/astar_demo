#ifndef NAVIGATION_STOPWATCH_H_
#define NAVIGATION_STOPWATCH_H_

#include <chrono>

class Stopwatch {
 public:
  Stopwatch(bool run = true) {
    if (run) {
      start_ = std::chrono::steady_clock::now();
    }
  }

  void reset() {
    start_ = std::chrono::steady_clock::now();
  }

  std::chrono::milliseconds elapsed() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_);
  }

  template <typename Duration>
  bool elapsed(Duration duration) {
    return std::chrono::steady_clock::now() - start_ > duration;
  }
 private:
  std::chrono::steady_clock::time_point start_;
};

#endif