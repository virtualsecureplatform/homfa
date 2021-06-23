#include "offline_dfa.hpp"

#include <execution>

#include <spdlog/spdlog.h>

OfflineDFARunner::OfflineDFARunner(const Graph &graph,
                                   InputStream<TRGSWLvl1FFT> &input_stream,
                                   std::shared_ptr<GateKey> gate_key)
    : graph_(graph),
      input_stream_(input_stream),
      weight_(graph_.size()),
      has_evaluated_(false),
      gate_key_(std::move(gate_key))
{
    for (Graph::State st = 0; st < graph_.size(); st++) {
        if (graph_.is_final_state(st))
            weight_.at(st).s = RedundantTRLWELvl1::TRIVIAL_1;
        else
            weight_.at(st).s = RedundantTRLWELvl1::TRIVIAL_0;
    }

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

TLWELvl1 OfflineDFARunner::result() const
{
    assert(has_evaluated_);
    RedundantTRLWELvl1 w = weight_.at(graph_.initial_state());
    switch (w.s) {
    case RedundantTRLWELvl1::TRIVIAL_0:
        return trivial_TLWELvl1_minus_1over8();
    case RedundantTRLWELvl1::TRIVIAL_1:
        return trivial_TLWELvl1_1over8();
    default:
        break;
    }

    if (gate_key_)
        do_SEI_IKS_GBTLWE2TRLWE(w.c, *gate_key_);
    TLWELvl1 ret;
    TFHEpp::SampleExtractIndex<Lvl1>(ret, w.c, 0);
    return ret;
}

void OfflineDFARunner::eval()
{
    assert(!has_evaluated_);
    has_evaluated_ = true;

    const TRLWELvl1 tri0 = trivial_TRLWELvl1_minus_1over8(),
                    tri1 = trivial_TRLWELvl1_1over8();
    auto remove_redundancy =
        [&](const RedundantTRLWELvl1 &r) -> const TRLWELvl1 & {
        switch (r.s) {
        case RedundantTRLWELvl1::TRIVIAL_0:
            return tri0;
        case RedundantTRLWELvl1::TRIVIAL_1:
            return tri1;
        default:
            return r.c;
        }
    };

    size_t input_size = input_stream_.size();
    std::vector<RedundantTRLWELvl1> out(weight_.size());
    for (int j = input_size - 1; j >= 0; --j) {
        auto states = graph_.states_at_depth(j);
        TRGSWLvl1FFT input = input_stream_.next();
        std::for_each(std::execution::par, states.begin(), states.end(),
                      [&](auto &&q) {
                          Graph::State q0 = graph_.next_state(q, false),
                                       q1 = graph_.next_state(q, true);
                          const RedundantTRLWELvl1 &rw0 = weight_.at(q0),
                                                   &rw1 = weight_.at(q1);

                          if (rw0.s != RedundantTRLWELvl1::NON_TRIVIAL &&
                              rw0.s == rw1.s) {
                              out.at(q).s = rw0.s;
                              return;
                          }

                          const TRLWELvl1 &w0 = remove_redundancy(rw0),
                                          &w1 = remove_redundancy(rw1);
                          TFHEpp::CMUXFFT<Lvl1>(out.at(q).c, input, w1, w0);
                          out.at(q).s = RedundantTRLWELvl1::NON_TRIVIAL;
                      });
        {
            using std::swap;
            swap(out, weight_);
        }

        if (size_t cur = input_size - j;
            gate_key_ && cur != 0 && cur % BOOT_INTERVAL == 0) {
            spdlog::info("Bootstrapping occurred");
            bootstrap_weight(states);
        }
    }
}

void OfflineDFARunner::bootstrap_weight(
    const std::vector<Graph::State> &targets)
{
    assert(gate_key_);
    std::for_each(std::execution::par, targets.begin(), targets.end(),
                  [&](Graph::State q) {
                      RedundantTRLWELvl1 &rw = weight_.at(q);
                      if (rw.s == RedundantTRLWELvl1::NON_TRIVIAL)
                          do_SEI_IKS_GBTLWE2TRLWE(rw.c, *gate_key_);
                  });
}
