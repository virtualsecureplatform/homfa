#include "online_dfa.hpp"

#include <execution>

#include <spdlog/spdlog.h>

/* OnlineDFARunner */
OnlineDFARunner::OnlineDFARunner(const Graph &graph,
                                 std::shared_ptr<GateKey> gate_key)
    : graph_(graph),
      weight_(graph.size(), trivial_TRLWELvl1_zero()),
      gate_key_(std::move(gate_key)),
      bootstrap_interval_(0),
      num_processed_inputs_(0)
{
    for (Graph::State st = 0; st < graph_.size(); st++)
        if (st == graph_.initial_state())
            weight_.at(st)[1][0] = (1u << 29);  // 1/8
        else
            weight_.at(st)[1][0] = -(1u << 29);  // -1/8

    // log_n(8000) = log(8000)/log(n)
    assert(graph_.size() > 1);
    bootstrap_interval_ = std::log(8000) / std::log(graph_.size());
    assert(bootstrap_interval_ > 0);

    spdlog::info("Parameter:");
    spdlog::info("\tMode:\t{}", "Online FA Runner");
    spdlog::info("\tState size:\t{}", graph_.size());
    spdlog::info("\tWeight size:\t{}", weight_.size());
    spdlog::info("\tConcurrency:\t{}", std::thread::hardware_concurrency());
    spdlog::info("\tBootstrap interval:\t{}", bootstrap_interval_);
    spdlog::info("");
}

TLWELvl1 OnlineDFARunner::result()
{
    if (gate_key_)
        bootstrap_weight();  // FIXME: is this necessary?
    TRLWELvl1 acc = trivial_TRLWELvl1_minus_1over8(),
              offset = trivial_TRLWELvl1_1over8();
    for (Graph::State st = 0; st < graph_.size(); st++) {
        if (graph_.is_final_state(st)) {
            TRLWELvl1_add(acc, weight_.at(st));
            TRLWELvl1_add(acc, offset);
        }
    }
    if (gate_key_)
        do_SEI_IKS_GBTLWE2TRLWE(acc, *gate_key_);
    TLWELvl1 ret;
    TFHEpp::SampleExtractIndex<Lvl1>(ret, acc, 0);
    return ret;
}

void OnlineDFARunner::eval_one(const TRGSWLvl1FFT &input)
{
    std::vector<TRLWELvl1> out{weight_.size()};
    std::vector<Graph::State> states = graph_.all_states();
    std::for_each(
        std::execution::par, states.begin(), states.end(),
        [&](Graph::State st) {
            std::vector<Graph::State> parents0 = graph_.prev_states(st, false),
                                      parents1 = graph_.prev_states(st, true);
            TRLWELvl1 acc0 = trivial_TRLWELvl1_minus_1over8(),
                      acc1 = trivial_TRLWELvl1_minus_1over8(),
                      offset = trivial_TRLWELvl1_1over8();
            for (Graph::State p0 : parents0) {
                TRLWELvl1_add(acc0, weight_.at(p0));
                TRLWELvl1_add(acc0, offset);
            }
            for (Graph::State p1 : parents1) {
                TRLWELvl1_add(acc1, weight_.at(p1));
                TRLWELvl1_add(acc1, offset);
            }
            TFHEpp::CMUXFFT<Lvl1>(out.at(st), input, acc1, acc0);
        });
    {
        using std::swap;
        swap(out, weight_);
    }

    if (++num_processed_inputs_ % bootstrap_interval_ == 0)
        bootstrap_weight();
}

void OnlineDFARunner::bootstrap_weight()
{
    assert(gate_key_);
    std::for_each(
        std::execution::par, weight_.begin(), weight_.end(),
        [&](TRLWELvl1 &w) { do_SEI_IKS_GBTLWE2TRLWE(w, *gate_key_); });
}

/* OnlineDFARunner2 */
OnlineDFARunner2::OnlineDFARunner2(const Graph &graph,
                                   std::shared_ptr<GateKey> gate_key)
    : graph_(graph),
      weight_(graph.size(), trivial_TRLWELvl1_zero()),
      gate_key_(gate_key),
      num_processed_inputs_(0)
{
    for (Graph::State st = 0; st < graph_.size(); st++)
        if (graph_.is_final_state(st))
            weight_.at(st)[1][0] = (1u << 29);  // 1/8
        else
            weight_.at(st)[1][0] = -(1u << 29);  // -1/8

    spdlog::info("Parameter:");
    spdlog::info("\tMode:\t{}", "Online FA Runner2");
    spdlog::info("\tState size:\t{}", graph_.size());
    spdlog::info("\tWeight size:\t{}", weight_.size());
    spdlog::info("\tConcurrency:\t{}", std::thread::hardware_concurrency());
    spdlog::info("");
}

TLWELvl1 OnlineDFARunner2::result() const
{
    TRLWELvl1 w = weight_.at(graph_.initial_state());
    if (gate_key_)
        do_SEI_IKS_GBTLWE2TRLWE(w, *gate_key_);
    TLWELvl1 ret;
    TFHEpp::SampleExtractIndex<Lvl1>(ret, w, 0);
    return ret;
}

void OnlineDFARunner2::eval_one(const TRGSWLvl1FFT &input)
{
    std::vector<TRLWELvl1> out(weight_.size(), trivial_TRLWELvl1_zero());
    std::vector<Graph::State> states = graph_.all_states();
    std::for_each(
        std::execution::par, states.begin(), states.end(), [&](Graph::State q) {
            const TRLWELvl1 &w0 = weight_.at(graph_.next_state(q, false)),
                            &w1 = weight_.at(graph_.next_state(q, true));
            TFHEpp::CMUXFFT<Lvl1>(out.at(q), input, w1, w0);
        });
    {
        using std::swap;
        swap(out, weight_);
    }
    if (++num_processed_inputs_ % BOOT_INTERVAL == 0) {
        spdlog::info("Bootstrapping occurred");
        bootstrap_weight();
    }
}

void OnlineDFARunner2::bootstrap_weight()
{
    assert(gate_key_);
    std::for_each(
        std::execution::par, weight_.begin(), weight_.end(),
        [&](TRLWELvl1 &w) { do_SEI_IKS_GBTLWE2TRLWE(w, *gate_key_); });
}
