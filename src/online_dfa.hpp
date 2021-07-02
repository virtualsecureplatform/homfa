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

class OnlineDFARunner3 {
private:
    const Graph &graph_;
    const GateKey &gate_key_;
    const TFHEpp::TLWE2TRLWEIKSKey<TFHEpp::lvl11param> &tlwel1_trlwel1_iks_key_;
    std::vector<TRLWELvl1> weight_;
    std::vector<TRGSWLvl1FFT> queued_inputs_;
    size_t first_lut_max_depth_, queue_size_;

    std::optional<SecretKey> debug_skey_;

    // Workspace for eval_queued_inputs()
    std::vector<TRLWELvl1> workspace_table1_, workspace_table2_;

public:
    OnlineDFARunner3(const Graph &graph, size_t first_lut_max_depth,
                     const GateKey &gate_key,
                     const TFHEpp::TLWE2TRLWEIKSKey<TFHEpp::lvl11param>
                         &tlwel1_trlwel1_iks_key,
                     std::optional<SecretKey> debug_skey);

    size_t queue_size() const
    {
        return queue_size_;
    }
    size_t first_lut_max_depth() const
    {
        return first_lut_max_depth_;
    }
    size_t second_lut_max_depth() const
    {
        return queue_size_ - first_lut_max_depth();
    }

    TLWELvl1 result();
    void eval_one(const TRGSWLvl1FFT &input);

private:
    void eval_queued_inputs();
};

#endif
