#ifndef HOMFA_BACKSTREAM_DFA_RUNNER_HPP
#define HOMFA_BACKSTREAM_DFA_RUNNER_HPP

#include "graph.hpp"
#include "tfhepp_util.hpp"

class BackstreamDFARunner {
private:
    Graph graph_;
    std::vector<RedundantTRLWELvl1> weight_;
    std::shared_ptr<GateKey> gate_key_;
    std::optional<size_t> input_size_;
    const size_t boot_interval_;
    size_t num_processed_inputs_;
    const TRLWELvl1 trlwelvl1_trivial_0_, trlwelvl1_trivial_1_;

    // Workspace for eval
    std::vector<RedundantTRLWELvl1> workspace_;

public:
    BackstreamDFARunner(Graph graph, size_t boot_interval,
                        std::optional<size_t> input_size = std::nullopt,
                        std::shared_ptr<GateKey> gate_key = nullptr);

    const Graph &graph() const
    {
        return graph_;
    }

    TLWELvl1 result() const;
    void eval(const TRGSWLvl1FFT &input);
    void eval_all(InputStream<TRGSWLvl1FFT> &input_stream);

private:
    const TRLWELvl1 &remove_redundancy(const RedundantTRLWELvl1 &src);
    void bootstrap_weight(const std::vector<Graph::State> &targets);
};

#endif
