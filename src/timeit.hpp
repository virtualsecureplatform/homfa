#ifndef HOMFA_TIMEIT_HPP
#define HOMFA_TIMEIT_HPP

#include <chrono>
#include <functional>
#include <iosfwd>

// Use the different translation unit in order to prohibit inline expansion of
// timeit()
std::chrono::microseconds timeit(std::function<void()> f);

class TimeRecorder {
public:
    enum class TARGET {
        CIRCUIT_BOOTSTRAPPING,
        BOOTSTRAPPING,
        CMUX,
    };

private:
    struct Sample {
        TARGET target;                   // What operation
        size_t count;                    // How many times evaluated
        std::chrono::microseconds time;  // Duration
    };
    std::vector<Sample> samples_;

public:
    TimeRecorder();

    void timeit(TARGET target, size_t count, std::function<void()> f);
    void clear();
    void dumpCSV(std::ostream& os) const;
};

#endif
