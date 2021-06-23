#ifndef HOMFA_ONLINE_DFA_HPP
#define HOMFA_ONLINE_DFA_HPP

#include "backstream_dfa_runner.hpp"
#include "graph.hpp"
#include "tfhepp_util.hpp"

class OnlineDFARunner {
private:
    const Graph &graph_;
    std::vector<TRLWELvl1> weight_;
    std::shared_ptr<GateKey> gate_key_;
    size_t bootstrap_interval_, num_processed_inputs_;

public:
    OnlineDFARunner(const Graph &graph,
                    std::shared_ptr<GateKey> gate_key = nullptr);

    TLWELvl1 result();
    void eval_one(const TRGSWLvl1FFT &input);

private:
    void bootstrap_weight();
};

class OnlineDFARunner2 {
private:
    BackstreamDFARunner runner_;

public:
    OnlineDFARunner2(const Graph &graph,
                     std::shared_ptr<GateKey> gate_key = nullptr);

    TLWELvl1 result() const;
    void eval_one(const TRGSWLvl1FFT &input);
};

#endif
