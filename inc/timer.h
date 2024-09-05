#ifndef TIMER_H
#define TIMER_H

#pragma once
#include <chrono>
// TODO Maybe we can just return a std::chrono::duration
class Timer {
public:
  Timer() : start_time_() {}

  void start() { start_time_ = std::chrono::high_resolution_clock::now(); }

  template <class Units> inline 
  Units elapsed_time() {
    return elapsed_time_<Units>();
  }

  template<class Unit> inline
  Unit get_time_sec() {
     return get_time<Unit>();
  }

private:
  std::chrono ::time_point<std::chrono::high_resolution_clock> start_time_;

  template <class Units> inline 
  Units elapsed_time_() {
    return std::chrono::duration_cast<Units>(
               std::chrono::high_resolution_clock::now() - start_time_);
  }

  template <class Units> inline 
  Units get_time() {
    return std::chrono::time_point_cast<Units>(
               std::chrono::high_resolution_clock::now())
        .time_since_epoch();
  }
};

#endif // TIMER_H
