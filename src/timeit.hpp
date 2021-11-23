#ifndef HOMFA_TIMEIT_HPP
#define HOMFA_TIMEIT_HPP

#include <chrono>
#include <functional>

// Use the different translation unit in order to prohibit inline expansion of
// timeit()
std::chrono::microseconds timeit(std::function<void()> f);

#endif
