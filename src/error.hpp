#ifndef HOMFA_ERROR_HPP
#define HOMFA_ERROR_HPP

#include <iostream>
#include <sstream>
#include <vector>

#include <backward.hpp>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace error {
inline void initialize(const std::string& tag)
{
    spdlog::set_level(spdlog::level::info);
    spdlog::drop_all();
    spdlog::set_default_logger(spdlog::stderr_color_mt(tag));
    // Use ISO 8601 for the datetime format
    spdlog::set_pattern("[%Y-%m-%dT%T.%e%z] [%n] [%^%l%$]\t%v");
}

template <class... Args>
[[noreturn]] void die(Args... args)
{
    using namespace backward;

    spdlog::error(std::forward<Args>(args)...);

#ifndef NDEBUG
    {
        // Print backtrace
        spdlog::error("Preparing backtrace...");
        std::stringstream ss;
        StackTrace st;
        st.load_here(32);
        Printer p;
        p.print(st, ss);
        spdlog::error(ss.str());
    }
#endif

    // Abort
    std::exit(EXIT_FAILURE);
}

class Stack {
private:
    std::vector<std::string> msgs_;

public:
    template <class... Args>
    void add(Args... args)
    {
        std::stringstream ss;
        (ss << ... << args);
        msgs_.push_back(ss.str());
    }

    bool empty() const
    {
        return msgs_.empty();
    }

    std::string str() const
    {
        std::stringstream ss;
        for (auto&& msg : msgs_)
            ss << msg << std::endl;
        return ss.str();
    }
};

}  // namespace error

#endif
