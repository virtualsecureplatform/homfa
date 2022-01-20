#ifndef HOMFA_ONLINE_DFA_HPP
#define HOMFA_ONLINE_DFA_HPP

#include "backstream_dfa_runner.hpp"
#include "graph.hpp"
#include "tfhepp_util.hpp"
#include "timeit.hpp"

class OnlineDFARunner2 {
private:
    BackstreamDFARunner runner_;

public:
    OnlineDFARunner2(const Graph &graph, size_t boot_interval_,
                     bool is_spec_reversed, std::shared_ptr<GateKey> gate_key,
                     bool sanitize_result);

    const Graph &graph() const
    {
        return runner_.graph();
    }

    const TimeRecorder &timer() const
    {
        return runner_.timer();
    }

    TLWELvl1 result() const;
    void eval_one(const TRGSWLvl1FFT &input);
};

class OnlineDFARunner4 {
private:
    Graph graph_;
    const GateKey &gate_key_;
    const CircuitKey &circuit_key_;
    size_t queue_size_;
    std::vector<TRGSWLvl1FFT> queued_inputs_;
    std::optional<TRLWELvl1> selector_;
    std::vector<Graph::State> live_states_;
    bool sanitize_result_;

    std::vector<TRLWELvl1> workspace1_, workspace2_;
    std::vector<TRGSWLvl1FFT> workspace3_;
    std::vector<TLWELvl0> workspace4_;

    TimeRecorder timer_;

public:
    OnlineDFARunner4(Graph graph, size_t queue_size, const GateKey &gate_key,
                     const CircuitKey &circuit_key, bool sanitize_result);

    const Graph &graph() const
    {
        return graph_;
    }

    size_t queue_size() const
    {
        return queue_size_;
    }

    size_t num_live_states() const
    {
        return live_states_.size();
    }

    const TimeRecorder &timer() const
    {
        return timer_;
    }

    TLWELvl1 result();
    void eval_one(const TRGSWLvl1FFT &input);

private:
    void eval_queued_inputs();
};

#endif
