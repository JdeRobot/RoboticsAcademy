#ifndef INCLUDE_LAP_HPP_
#define INCLUDE_LAP_HPP_

#include <chrono>
#include <iostream>
#include <thread>
#include <string>
#include <cstdio>

using namespace std;  // NOLINT

class Lap
{
private:
  chrono::milliseconds lap_time;
  std::chrono::system_clock::time_point start_time;

  bool paused = false;

public:
  Lap();
  ~Lap();

  void pause();
  void start();
  void reset();
  std::string getLapTime();
};

Lap::Lap()
{
  this->reset();
}

Lap::~Lap()
{
}

std::string Lap::getLapTime()
{
  if (!paused)
  {
    const auto lapsed_time = chrono::duration_cast<chrono::milliseconds>(
      chrono::high_resolution_clock::now() - start_time);
    if (lapsed_time < 1000ms) {
      lap_time += lapsed_time;
    }
  }

  start_time = chrono::high_resolution_clock::now();

  char buff[70];
  const int sec = lap_time.count() / 1000;
  const int min = sec / 60;
  const int h = min / 60;
  sprintf(buff, "%d:%d:%d.%ld", h, min, sec, lap_time.count() % 1000);
  return std::string(buff);
}

void Lap::reset()
{
  start_time = chrono::high_resolution_clock::now();
  lap_time = chrono::milliseconds(0);
  paused = false;
  return;
}

void Lap::pause()
{
  paused = true;
  return;
}

void Lap::start()
{
  start_time = chrono::high_resolution_clock::now();
  paused = false;
  return;
}

#endif  // INCLUDE_LAP_HPP_
