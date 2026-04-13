#ifndef INCLUDE_LAP_HPP_
#define INCLUDE_LAP_HPP_

#include <string>
#include <chrono>
#include <memory>
#include "Map.hpp"

class Lap {
public:
    Lap(std::shared_ptr<Map> map_object);

    double check_threshold();
    double return_lap_time() const;
    void reset();
    void pause();
    void unpause();

private:
    std::shared_ptr<Map> map_;
    std::string target_start_;
    std::string target_end_;

    std::chrono::time_point<std::chrono::system_clock> start_time_;
    std::chrono::duration<double> lap_time_;
    bool buffer_;
    bool pause_condition_;
};

#endif