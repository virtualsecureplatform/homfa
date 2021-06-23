#include "backstream_dfa_runner.hpp"

#include <execution>

#include <spdlog/spdlog.h>

BackstreamDFARunner::BackstreamDFARunner(const Graph &graph,
                                         std::optional<size_t> input_size,
                                         std::shared_ptr<GateKey> gate_key)
    : graph_(graph),
      weight_(graph_.size()),
      gate_key_(std::move(gate_key)),
      input_size_(std::move(input_size)),
      num_processed_inputs_(0),
      trlwelvl1_trivial_0_(trivial_TRLWELvl1_minus_1over8()),
      trlwelvl1_trivial_1_(trivial_TRLWELvl1_1over8())
{
    for (Graph::State st = 0; st < graph_.size(); st++) {
        if (graph_.is_final_state(st))
            weight_.at(st).s = RedundantTRLWELvl1::TRIVIAL_1;
        else
            weight_.at(st).s = RedundantTRLWELvl1::TRIVIAL_0;
    }
}

TLWELvl1 BackstreamDFARunner::result() const
{
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

void BackstreamDFARunner::eval(const TRGSWLvl1FFT &input)
{
    std::vector<RedundantTRLWELvl1> out(weight_.size());
    std::optional<std::vector<Graph::State>> states;
    if (input_size_) {
        int j = *input_size_ - num_processed_inputs_;
        assert(j > 0);
        states.emplace(graph_.states_at_depth(j - 1));
    }
    else {
        states.emplace(graph_.all_states());
    }

    std::for_each(
        std::execution::par, states->begin(), states->end(),
        [&](Graph::State q) {
            Graph::State q0 = graph_.next_state(q, false),
                         q1 = graph_.next_state(q, true);
            const RedundantTRLWELvl1 &rw0 = weight_.at(q0),
                                     &rw1 = weight_.at(q1);

            if (rw0.s != RedundantTRLWELvl1::NON_TRIVIAL && rw0.s == rw1.s) {
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

    num_processed_inputs_++;
    if (gate_key_ && num_processed_inputs_ % BOOT_INTERVAL == 0) {
        spdlog::debug("Bootstrapping occurred");
        bootstrap_weight(*states);
    }
}

const TRLWELvl1 &BackstreamDFARunner::remove_redundancy(
    const RedundantTRLWELvl1 &src)
{
    switch (src.s) {
    case RedundantTRLWELvl1::TRIVIAL_0:
        return trlwelvl1_trivial_0_;
    case RedundantTRLWELvl1::TRIVIAL_1:
        return trlwelvl1_trivial_1_;
    default:
        return src.c;
    }
}

void BackstreamDFARunner::bootstrap_weight(
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
