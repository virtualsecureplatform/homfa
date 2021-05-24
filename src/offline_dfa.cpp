#include "offline_dfa.hpp"

#include <execution>

#include <spdlog/spdlog.h>

OfflineFARunner::OfflineFARunner(const Graph &graph,
                                 InputStream<TRGSWLvl1FFT> &input_stream,
                                 std::shared_ptr<GateKey> gate_key)
    : graph_(graph),
      input_stream_(input_stream),
      weight_(graph_.size(), trivial_TRLWELvl1_zero()),
      has_evaluated_(false),
      gate_key_(std::move(gate_key))
{
    for (Graph::State st = 0; st < graph_.size(); st++)
        if (graph_.is_final_state(st))
            weight_.at(st)[1][0] = (1u << 29);  // 1/8
        else
            weight_.at(st)[1][0] = -(1u << 29);  // -1/8

    spdlog::info("Parameter:");
    spdlog::info("\tMode:\t{}", "Offline FA Runner");
    spdlog::info("\tInput size:\t{}", input_stream.size());
    spdlog::info("\tState size:\t{}", graph_.size());
    spdlog::info("\tWeight size:\t{}", weight_.size());
    spdlog::info("\tConcurrency:\t{}", std::thread::hardware_concurrency());

    {
        size_t total_cnt_cmux = 0;
        for (size_t j = 0; j < input_stream_.size(); j++)
            total_cnt_cmux += graph_.states_at_depth(j).size();
        spdlog::info("\tTotal #CMUX:\t{}", total_cnt_cmux);
    }

    spdlog::info("");
}

TLWELvl1 OfflineFARunner::result() const
{
    assert(has_evaluated_);
    TRLWELvl1 w = weight_.at(graph_.initial_state());
    if (gate_key_)
        do_SEI_IKS_GBTLWE2TRLWE(w, *gate_key_);
    TLWELvl1 ret;
    TFHEpp::SampleExtractIndex<Lvl1>(ret, w, 0);
    return ret;
}

void OfflineFARunner::eval()
{
    assert(!has_evaluated_);
    has_evaluated_ = true;

    size_t input_size = input_stream_.size();
    std::vector<TRLWELvl1> out(weight_.size(), trivial_TRLWELvl1_zero());
    for (int j = input_size - 1; j >= 0; --j) {
        auto states = graph_.states_at_depth(j);
        TRGSWLvl1FFT input = input_stream_.next();
        std::for_each(std::execution::par, states.begin(), states.end(),
                      [&](auto &&q) {
                          TRLWELvl1 w0, w1;
                          next_weight(w1, j, q, true);
                          next_weight(w0, j, q, false);
                          TFHEpp::CMUXFFT<Lvl1>(out.at(q), input, w1, w0);
                      });
        {
            using std::swap;
            swap(out, weight_);
        }

        if (size_t cur = input_size - j;
            gate_key_ && cur != 0 && cur % BOOT_INTERVAL == 0) {
            spdlog::info("Bootstrapping occurred");
            bootstrapping_of_weight();
        }

        spdlog::debug("[{}] #CMUX : {}", states.size());
    }
}

void OfflineFARunner::next_weight(TRLWELvl1 &out, int j, Graph::State from,
                                  bool input) const
{
    Graph::State to = graph_.next_state(from, input);
    out = weight_.at(to);
}

void OfflineFARunner::bootstrapping_of_weight()
{
    assert(gate_key_);
    std::for_each(
        std::execution::par, weight_.begin(), weight_.end(),
        [&](TRLWELvl1 &w) { do_SEI_IKS_GBTLWE2TRLWE(w, *gate_key_); });
}
