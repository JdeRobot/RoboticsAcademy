#ifndef INCLUDE_FREQUENCY_HPP_
#define INCLUDE_FREQUENCY_HPP_

#include <chrono>
#include <cmath>
#include <thread>

class Frequency
{
private:
  std::chrono::high_resolution_clock::time_point last_time;
  int is_first_iteration;

public:
  Frequency();
  ~Frequency();

  static int rate;
  void tick(int ideal_cycle = 50);
};

#endif