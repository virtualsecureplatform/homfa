#include "offline_dfa.hpp"

#include <execution>

#include <spdlog/spdlog.h>

OfflineDFARunner::OfflineDFARunner(Graph graph, size_t input_size,
                                   size_t boot_interval,
                                   std::shared_ptr<GateKey> gate_key)
    : runner_(std::move(graph), boot_interval, input_size, gate_key)
{
}

TLWELvl1 OfflineDFARunner::result() const
{
    return runner_.result();
}

void OfflineDFARunner::eval_one(const TRGSWLvl1FFT& input)
{
    runner_.eval(input);
}
