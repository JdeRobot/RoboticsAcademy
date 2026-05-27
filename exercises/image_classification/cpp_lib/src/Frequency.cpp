#include "Frequency.hpp"

int Frequency::rate = 0;

Frequency::Frequency() : is_first_iteration(1) {}

Frequency::~Frequency() {}

void Frequency::tick(int ideal_cycle)
{
  const auto ideal_ms = std::chrono::milliseconds(1000 / ideal_cycle);
  const auto now = std::chrono::high_resolution_clock::now();

  if (is_first_iteration == 1)
  {
    last_time = now;
    is_first_iteration = 0;
    return;
  }

  const auto iter_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time);

  if (iter_ms < ideal_ms)
  {
    rate = std::round(1000.0 / ideal_ms.count());
    std::this_thread::sleep_for(ideal_ms - iter_ms);
  }
  else
  {
    rate = std::round(1000.0 / iter_ms.count());
  }

  last_time = now;
}
