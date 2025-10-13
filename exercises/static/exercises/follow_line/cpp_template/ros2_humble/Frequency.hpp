#ifndef INCLUDE_FREQUENCY_HPP_
#define INCLUDE_FREQUENCY_HPP_

#include <chrono>
#include <iostream>
#include <thread>
#include <bits/stdc++.h>

using namespace std;

class Frequency
{
private:
  chrono::_V2::system_clock::time_point last_time;
  int is_first_iteration;

public:
  Frequency(/* args */);
  ~Frequency();

  static int rate;
  void tick(int ideal_cycle);
};

Frequency::Frequency(/* args */)
{
  is_first_iteration = 1;
}

Frequency::~Frequency()
{
}

void Frequency::tick(int ideal_cycle = 50)
{
  const auto ideal_ms = chrono::milliseconds(1000 / ideal_cycle);
  const auto now = chrono::high_resolution_clock::now();

  if (is_first_iteration == 1)
  {
    last_time = now;
    is_first_iteration = 0;
    return;
  }

  const auto iter_ms = chrono::duration_cast<chrono::milliseconds>(now - last_time);

  if (iter_ms < ideal_ms)
  {
    rate = round(1000 / ideal_ms.count());
    this_thread::sleep_for(chrono::milliseconds(ideal_ms - iter_ms));
  }
  else
  {
    rate = round(1000 / iter_ms.count());
  }

  last_time = now;
  return;
}

int Frequency::rate = 0;

#endif