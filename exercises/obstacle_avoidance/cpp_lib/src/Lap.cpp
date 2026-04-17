#include "Lap.hpp"

Lap::Lap(std::shared_ptr<Map> map_object) 
    : map_(map_object), target_start_("target01"), target_end_("NaN") {
    reset();
}

double Lap::check_threshold() {
    auto target = map_->getNextTarget();
    if (!target) return lap_time_.count();

    std::string targetid = target->getId();

    if (targetid != target_end_ && targetid != target_start_) {
        if (buffer_) {
            start_time_ = std::chrono::system_clock::now();
            buffer_ = false;
        }

        if (!pause_condition_) {
            auto now = std::chrono::system_clock::now();
            if (lap_time_.count() == 0.0) {
                lap_time_ = now - start_time_;
            } else {
                lap_time_ += now - start_time_;
            }
            start_time_ = now;
        }
    }

    return lap_time_.count();
}

double Lap::return_lap_time() const {
    return lap_time_.count();
}

void Lap::reset() {
    lap_time_ = std::chrono::duration<double>::zero();
    buffer_ = true;
    pause_condition_ = false;
}

void Lap::pause() {
    pause_condition_ = true;
}

void Lap::unpause() {
    if (pause_condition_) {
        start_time_ = std::chrono::system_clock::now();
    }
    pause_condition_ = false;
}