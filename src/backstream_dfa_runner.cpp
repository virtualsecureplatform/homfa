#include "backstream_dfa_runner.hpp"
#include "error.hpp"

#include <execution>

#include <spdlog/spdlog.h>

BackstreamDFARunner::BackstreamDFARunner(Graph graph, size_t boot_interval,
                                         std::optional<size_t> input_size,
                                         std::shared_ptr<GateKey> gate_key,
                                         bool sanitize_result)
    : graph_(std::move(graph)),
      weight_(graph_.size()),
      gate_key_(std::move(gate_key)),
      input_size_(std::move(input_size)),
      boot_interval_(boot_interval),
      num_processed_inputs_(0),
      trlwelvl1_trivial_0_(trivial_TRLWELvl1_zero()),
      trlwelvl1_trivial_1_(trivial_TRLWELvl1_1over2()),
      sanitize_result_(sanitize_result),
      workspace_(graph_.size())
{
    assert(gate_key_);

    if (sanitize_result_)
        error::die("Sanitization of results is not implemented");

    if (input_size_)
        graph_.reserve_states_at_depth(*input_size_);

    for (Graph::State st = 0; st < graph_.size(); st++)
        weight_.at(st) = graph_.is_final_state(st) ? trlwelvl1_trivial_1_
                                                   : trlwelvl1_trivial_0_;
}

TLWELvl1 BackstreamDFARunner::result() const
{
    TLWELvl1 ret;
    TFHEpp::SampleExtractIndex<Lvl1>(ret, weight_.at(graph_.initial_state()),
                                     0);
    return ret;
}

void BackstreamDFARunner::eval(const TRGSWLvl1FFT &input)
{
    std::vector<TRLWELvl1> &out = workspace_;
    out.resize(graph_.size());

    std::optional<std::vector<Graph::State>> states;
    if (input_size_) {
        int j = *input_size_ - num_processed_inputs_;
        assert(j > 0);
        states.emplace(graph_.states_at_depth(j - 1));
    }
    else {
        states.emplace(graph_.all_states());
    }

    std::for_each(std::execution::par, states->begin(), states->end(),
                  [&](Graph::State q) {
                      Graph::State q0 = graph_.next_state(q, false),
                                   q1 = graph_.next_state(q, true);
                      const TRLWELvl1 &w0 = weight_.at(q0),
                                      &w1 = weight_.at(q1);
                      TFHEpp::CMUXFFT<Lvl1>(out.at(q), input, w1, w0);
                  });
    {
        using std::swap;
        swap(out, weight_);
    }

    num_processed_inputs_++;
    if (gate_key_ && num_processed_inputs_ % boot_interval_ == 0) {
        spdlog::debug("Bootstrapping occurred");
        bootstrap_weight(*states);
    }
}

void BackstreamDFARunner::bootstrap_weight(
    const std::vector<Graph::State> &targets)
{
    assert(gate_key_);
    std::for_each(std::execution::par, targets.begin(), targets.end(),
                  [&](Graph::State q) {
                      TRLWELvl1 &w = weight_.at(q);
                      do_SEI_IKS_GBTLWE2TRLWE_2(w, *gate_key_);
                  });
}
