#ifndef HOMFA_ONLINE_DFA_HPP
#define HOMFA_ONLINE_DFA_HPP

#include "backstream_dfa_runner.hpp"
#include "graph.hpp"
#include "tfhepp_util.hpp"

class OnlineDFARunner {
private:
    Graph graph_;
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
    OnlineDFARunner2(const Graph &graph, size_t boot_interval_,
                     bool is_spec_reversed,
                     std::shared_ptr<GateKey> gate_key = nullptr);

    const Graph &graph() const
    {
        return runner_.graph();
    }

    TLWELvl1 result() const;
    void eval_one(const TRGSWLvl1FFT &input);
};

class OnlineDFARunner3 {
private:
    Graph graph_;
    const GateKey &gate_key_;
    const TFHEpp::TLWE2TRLWEIKSKey<TFHEpp::lvl11param> &tlwel1_trlwel1_iks_key_;
    std::vector<TRLWELvl1> weight_;
    std::vector<TRGSWLvl1FFT> queued_inputs_;
    size_t max_second_lut_depth_, queue_size_;
    std::vector<Graph::State> live_states_;
    std::vector<std::vector<Graph::State>> memo_transition_;
    size_t num_eval_, bootstrapping_freq_;

    std::optional<SecretKey> debug_skey_;

    // Workspace for eval_queued_inputs()
    std::vector<TRLWELvl1> workspace_table1_, workspace_table2_;

public:
    OnlineDFARunner3(Graph graph, size_t max_second_lut_depth,
                     size_t queue_size, size_t bootstrapping_freq,
                     const GateKey &gate_key,
                     const TFHEpp::TLWE2TRLWEIKSKey<TFHEpp::lvl11param>
                         &tlwel1_trlwel1_iks_key,
                     std::optional<SecretKey> debug_skey);

    size_t queue_size() const
    {
        return queue_size_;
    }

    TLWELvl1 result();
    void eval_one(const TRGSWLvl1FFT &input);

private:
    void eval_queued_inputs();
};

#endif
