#include "timeit.hpp"

#include <iostream>

std::chrono::microseconds timeit(std::function<void()> f)
{
    auto begin = std::chrono::high_resolution_clock::now();
    f();
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(end - begin);
}

// class TimeRecorder

TimeRecorder::TimeRecorder() : samples_()
{
}

void TimeRecorder::timeit(TARGET target, size_t count, std::function<void()> f)
{
    std::chrono::microseconds time = ::timeit(f);
    samples_.push_back(Sample{target, count, time});
}

void TimeRecorder::clear()
{
    samples_.clear();
}

void TimeRecorder::dumpCSV(std::ostream& os) const
{
    std::unordered_map<TARGET, Sample> summary;
    for (auto&& sample : samples_) {
        auto [it, inserted] = summary.emplace(sample.target, sample);
        if (inserted)
            continue;
        it->second.count += sample.count;
        it->second.time += sample.time;
    }

    for (auto&& [target, sample] : summary) {
        switch (target) {
        case TARGET::CIRCUIT_BOOTSTRAPPING:
            os << "time_recorder-circuit_bootstrapping";
            break;
        case TARGET::BOOTSTRAPPING:
            os << "time_recorder-bootstrapping";
            break;
        case TARGET::CMUX:
            os << "time_recorder-cmux";
            break;
        }
        os << "," << sample.count << "," << sample.time.count() << "\n";
    }
}
