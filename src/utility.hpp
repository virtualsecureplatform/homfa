#ifndef HOMFA_UTILITY_HPP
#define HOMFA_UTILITY_HPP

#include <cassert>

#include <fstream>
#include <string>

template <class Func>
void each_input_bit(const std::string& input_filename, size_t num_ap, Func func)
{
    std::ifstream ifs{input_filename};
    assert(ifs);

    size_t rest = 0;
    while (ifs) {
        int ch = ifs.get();
        if (ch == EOF)
            break;
        uint8_t v = ch;
        if (rest == 0)
            rest = num_ap;
        for (size_t i = 0; i < 8 && rest != 0; i++, rest--) {
            bool b = (v & 1u) != 0;
            v >>= 1;
            func(b);
        }
    }
    assert(rest == 0);
}

#endif
