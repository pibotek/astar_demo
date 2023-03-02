#ifndef NAVIGATION_STOPWATCH_H_
#define NAVIGATION_STOPWATCH_H_

#include <chrono>

using Clock = std::chrono::steady_clock;

class Stopwatch {
 public:
  Stopwatch(bool run = true) {
    if (run) {
      start_ = Clock::now();
    }
  }

  void reset() {
    start_ = Clock::now();
  }

  std::chrono::milliseconds elapsed() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start_);
  }

  template <typename Duration>
  bool elapsed(Duration duration) {
    return Clock::now() - start_ > duration;
  }

 private:
  Clock::time_point start_;
};

#endif
