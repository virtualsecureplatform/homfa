#ifndef HOMFA_BACKSTREAM_DFA_RUNNER_HPP
#define HOMFA_BACKSTREAM_DFA_RUNNER_HPP

#include "graph.hpp"
#include "tfhepp_util.hpp"
#include "timeit.hpp"

#include <optional>

class BackstreamDFARunner {
private:
    Graph graph_;
    std::vector<TRLWELvl1> weight_;
    std::shared_ptr<GateKey> gate_key_;
    std::optional<size_t> input_size_;
    const size_t boot_interval_;
    size_t num_processed_inputs_;
    const TRLWELvl1 trlwelvl1_trivial_0_, trlwelvl1_trivial_1_;
    bool sanitize_result_;

    // Workspace for eval
    std::vector<TRLWELvl1> workspace_;

    TimeRecorder timer_;

public:
    BackstreamDFARunner(Graph graph, size_t boot_interval,
                        std::optional<size_t> input_size,
                        std::shared_ptr<GateKey> gate_key,
                        bool sanitize_result);

    const Graph& graph() const
    {
        return graph_;
    }

    const TimeRecorder& timer() const
    {
        return timer_;
    }

    TLWELvl1 result() const;
    void eval(const TRGSWLvl1FFT& input);

private:
    void bootstrap_weight(const std::vector<Graph::State>& targets);
};

#endif
