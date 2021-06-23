#include "offline_dfa.hpp"

#include <execution>

#include <spdlog/spdlog.h>

OfflineDFARunner::OfflineDFARunner(const Graph &graph,
                                   InputStream<TRGSWLvl1FFT> &input_stream,
                                   std::shared_ptr<GateKey> gate_key)
    : runner_(graph, input_stream.size(), gate_key), input_stream_(input_stream)
{
    spdlog::info("Parameter:");
    spdlog::info("\tMode:\t{}", "Offline FA Runner");
    spdlog::info("\tInput size:\t{}", input_stream.size());
    spdlog::info("\tState size:\t{}", graph.size());
    spdlog::info("\tConcurrency:\t{}", std::thread::hardware_concurrency());

    {
        size_t total_cnt_cmux = 0;
        for (size_t j = 0; j < input_stream_.size(); j++)
            total_cnt_cmux += graph.states_at_depth(j).size();
        spdlog::info("\tTotal #CMUX:\t{}", total_cnt_cmux);
    }

    spdlog::info("");
}

TLWELvl1 OfflineDFARunner::result() const
{
    return runner_.result();
}

void OfflineDFARunner::eval()
{
    size_t input_size = input_stream_.size();
    for (int j = input_size - 1; j >= 0; --j) {
        TRGSWLvl1FFT input = input_stream_.next();
        runner_.eval(input);
    }
}
