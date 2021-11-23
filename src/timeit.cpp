#include "timeit.hpp"

std::chrono::microseconds timeit(std::function<void()> f)
{
    auto begin = std::chrono::high_resolution_clock::now();
    f();
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(end - begin);
}
